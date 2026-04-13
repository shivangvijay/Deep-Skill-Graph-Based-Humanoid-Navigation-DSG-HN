#pragma once

#include <svm.h>
#include <string>
#include <vector>
#include <stdexcept>

static void print_null(const char *s) {}

class InitiationSetClassifier
{
public:
    InitiationSetClassifier()
    {
        svm_set_print_string_function(&print_null);
    }

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
    // balance_classes: weight each class inversely by frequency — matches Python's SVC(class_weight='balanced')
    void train(const std::vector<std::vector<float>> &states,
               const std::vector<int> &labels,
               double C = 1.0, double gamma = -1.0, bool balance_classes = false)
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

        // Class balancing: weight each class inversely by its frequency
        // Matches Python's SVC(class_weight='balanced')
        std::vector<int> weight_labels;
        std::vector<double> weights;
        if (balance_classes)
        {
            int pos_count = std::count(labels.begin(), labels.end(), 1);
            int neg_count = std::count(labels.begin(), labels.end(), -1);
            int total = pos_count + neg_count;
            if (pos_count > 0 && neg_count > 0)
            {
                weight_labels = {1, -1};
                weights = {(double)total / (2.0 * pos_count),
                           (double)total / (2.0 * neg_count)};
                param.nr_weight = 2;
                param.weight_label = weight_labels.data();
                param.weight = weights.data();
            }
        }

        _make_problem(states, labels, false);

        if (_model)
            svm_free_and_destroy_model(&_model);
        _model = svm_train(&_p_two_class.prob, &param); // svm_train does not internally copy the problem data
    }

    // Train one-class SVM (positive examples only — no labels needed)
    void trainOneClass(const std::vector<std::vector<float>> &states, double nu = 0.1)
    {
        if (states.empty())
            throw std::runtime_error("InitiationSetClassifier: no training data");

        // these parameters are commonly used for one-class SVMs, but may require tuning based on the data distribution
        svm_parameter param{};
        param.svm_type = ONE_CLASS;
        param.kernel_type = RBF;
        param.nu = nu;
        param.gamma = 0.5; //0.005 ;//1.0 / states[0].size() / 100.0;
        param.eps = 1e-4;
        param.cache_size = 256;
        param.probability = 0;
        param.nr_weight = 0;

        // ONE_CLASS ignores labels; pass zeros
        std::vector<int> dummy_labels(states.size(), 0);
        _make_problem(states, dummy_labels, true);

        if (_model)
            svm_free_and_destroy_model(&_model);
        _model = svm_train(&_p_one_class.prob, &param);
    }

    // Returns the raw signed decision value (distance to hyperplane).
    // Positive = inside initiation set, negative = outside, near 0 = boundary.
    double decisionValue(const std::vector<float> &state) const
    {
        _check_model();
        auto nodes = _make_nodes(state);
        double dec_val = 0.0;
        svm_predict_values(_model, nodes.data(), &dec_val);
        return dec_val;
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

    bool trained() const { return _model != nullptr; }

    // Returns all support vectors as dense float vectors (length = n_features).
    // libsvm stores SVs in sparse svm_node format; this decodes them to plain vectors.
    // SVs are the literal boundary points of the learned SVM — used as spawn states
    // for Phase 3 refinement rather than approximating from training data.
    std::vector<std::vector<float>> getSupportVectors(int n_features) const
    {
        _check_model();
        std::vector<std::vector<float>> svs;
        svs.reserve(_model->l);
        for (int i = 0; i < _model->l; ++i)
        {
            std::vector<float> vec(n_features, 0.0f);
            for (const svm_node *n = _model->SV[i]; n->index != -1; ++n)
                vec[n->index - 1] = static_cast<float>(n->value);
            svs.push_back(std::move(vec));
        }
        return svs;
    }

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
    
    Problem _p_one_class; // keep as global to ensure it does not go out of scope
    Problem _p_two_class;


    void _make_problem(const std::vector<std::vector<float>> &states,
                          const std::vector<int> &labels, bool one_class = false)
    {
        Problem &p = one_class ? _p_one_class : _p_two_class;
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
    }
};
