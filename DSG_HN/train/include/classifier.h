#pragma once

#include <svm.h>
#include <string>
#include <vector>
#include <stdexcept>

class InitiationSetClassifier
{
public:
    InitiationSetClassifier() = default;

    ~InitiationSetClassifier()
    {
        if (_model)
            svm_free_and_destroy_model(&_model);
    }

    // Non-copyable — svm_model* ownership is unique, but it can be moved
    InitiationSetClassifier(const InitiationSetClassifier &) = delete;
    InitiationSetClassifier &operator=(const InitiationSetClassifier &) = delete;

    // Movable
    InitiationSetClassifier(InitiationSetClassifier &&other) noexcept
        : _model(other._model) { other._model = nullptr; }
    InitiationSetClassifier &operator=(InitiationSetClassifier &&other) noexcept
    {
        if (this != &other)
        {
            if (_model)
                svm_free_and_destroy_model(&_model);
            _model = other._model;
            other._model = nullptr;
        }
        return *this;
    }

    // Train from collected states. labels: +1 = in set, -1 = not in set
    void train(const std::vector<std::vector<float>> &states,
               const std::vector<int> &labels,
               double C = 1.0, double gamma = -1.0)
    {
        if (states.empty())
            throw std::runtime_error("InitiationSetClassifier: no training data");

        svm_parameter param{};
        param.svm_type = C_SVC;
        param.kernel_type = RBF;
        param.C = C;
        param.gamma = (gamma > 0.0) ? gamma : 1.0 / states[0].size(); // default gamma = 1/num_features
        param.eps = 1e-3;
        param.cache_size = 256;
        param.probability = 0;
        param.nr_weight = 0;

        Problem p = _make_problem(states, labels);
        if (_model)
            svm_free_and_destroy_model(&_model);
        _model = svm_train(&p.prob, &param); // svm_train internally copies the problem data, so we can safely let p go out of scope
    }

    // Returns true if state is in the initiation set
    bool classify(const std::vector<float> &state) const
    {
        _check_model();                  // Ensure model is trained or loaded
        auto nodes = _make_nodes(state); // svm_node array for prediction: libsvm represents feature vectors as sparse arrays of {index, value} pairs terminated by {-1, 0}, _make_nodes() handles this conversion from a plain std::vector<float>
        return svm_predict(_model, nodes.data()) > 0;
    }

    void save(const std::string &path) const
    {
        _check_model();
        if (svm_save_model(path.c_str(), _model) != 0)
            throw std::runtime_error("InitiationSetClassifier: failed to save to " + path);
    }

    void load(const std::string &path)
    {
        svm_model *m = svm_load_model(path.c_str());
        if (!m)
            throw std::runtime_error("InitiationSetClassifier: failed to load from " + path);
        if (_model)
            svm_free_and_destroy_model(&_model);
        _model = m;
    }

    bool trained() const { return _model != nullptr; } // Check if model is trained

private:
    svm_model *_model = nullptr;

    void _check_model() const
    {
        if (!_model)
            throw std::runtime_error("InitiationSetClassifier: model not trained or loaded");
    }

    std::vector<svm_node> _make_nodes(const std::vector<float> &state) const
    {
        std::vector<svm_node> nodes(state.size() + 1);
        for (size_t i = 0; i < state.size(); ++i)
            nodes[i] = {static_cast<int>(i) + 1, static_cast<double>(state[i])};
        nodes.back() = {-1, 0.0};
        return nodes;
    }

    struct Problem
    {
        svm_problem prob;
        std::vector<double> y;
        std::vector<svm_node *> x_ptrs;
        std::vector<std::vector<svm_node>> x_nodes;
    };

    Problem _make_problem(const std::vector<std::vector<float>> &states,
                          const std::vector<int> &labels)
    {
        Problem p;
        p.prob.l = static_cast<int>(states.size());
        p.y.resize(states.size());
        p.x_nodes.resize(states.size());
        p.x_ptrs.resize(states.size());

        for (size_t i = 0; i < states.size(); ++i)
        {
            p.y[i] = static_cast<double>(labels[i]);
            p.x_nodes[i] = _make_nodes(states[i]);
            p.x_ptrs[i] = p.x_nodes[i].data();
        }

        p.prob.y = p.y.data();
        p.prob.x = p.x_ptrs.data();
        return p;
    }
};
