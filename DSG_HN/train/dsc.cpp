#include "dsc.h"
#include "param.h"
#include "robot_bridge_train.h"
#include "environment.h"
#include <torch/torch.h>
#include <iostream>

#define SCENE_FILE "../config/scene/test_scene.xml"
#define POLICY_DIR "config/policy/velocity"
#define CONFIG_PATH "config/config.yaml"

// Pre-trained global option (oG) — TD3 policy from a prior training run.
#define OG_ACTOR   "best_actor.pt"
#define OG_CRITIC1 "best_critic_1.pt"
#define OG_CRITIC2 "best_critic_2.pt"

// test_scene arena bounds (metres). Obstacles span ~±3 m; add margin.
#define X_MIN -5.0f
#define X_MAX  5.0f
#define Y_MIN -5.0f
#define Y_MAX  5.0f

// Global goal: set this to the desired target position in the test_scene coordinate frame.
static const std::array<float, 3> GLOBAL_GOAL = {3.0f, 3.0f, 0.0f};

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    std::string rel_path = param::config["FSM"]["Velocity"]["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(rel_path);

    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available — training on GPU." << std::endl;
        device = torch::Device(torch::kCUDA);
    }

    auto robot_bridge = std::make_shared<RobotBridgeTrain>(
        SCENE_FILE, X_MIN, X_MAX, Y_MIN, Y_MAX, policy_dir, /*render=*/false);
    auto train_env = std::make_shared<TrainEnvironment>(robot_bridge, 1000);

    DeepSkillChaining::Config cfg;
    cfg.gestation_n        = 10; // number of successes to achieve during gestation phase of skill learning before moving on to classifier training - maybe change this to a % of last 10 episodes or something that captures improvement more directly?
    cfg.last_k             = 10; // only keep the last k successful states from each episode during gestation to encourage diversity in the initiation set. states are sampled at 50Hz, so 10 states = last 0.2s of successful trajectory per episode.
    cfg.refinement_eps     = 20; // after gestation, keep training the skill for this many additional episodes to refine the policy and classifier
    cfg.nu                 = 0.1; // outlier fraction for initial one-class SVM classifier (number of outliers = nu * gestation_n)
    cfg.max_skills         = 6; // maximum number of skills to chain before giving up (to prevent infinite loops in edge cases where start is not covered)

    DeepSkillChaining dsc(train_env, device, GLOBAL_GOAL, cfg);
    dsc.loadGlobalOption(OG_ACTOR, OG_CRITIC1, OG_CRITIC2);
    int n = dsc.train();
    std::cout << "\nTraining complete: " << n << " skill(s) discovered." << std::endl;

    dsc.save("dsc_models");

    std::cout << "\n=== Evaluation (20 episodes) ===" << std::endl;
    float total = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        float r = dsc.execute();
        total += r;
        std::cout << "  Episode " << i + 1 << ": reward = " << r << std::endl;
    }
    std::cout << "Mean reward: " << (total / 20.0f) << std::endl;

    return 0;
}
