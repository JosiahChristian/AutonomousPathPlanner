#include "PlannerEngine.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class AcceptAllGate final : public DecisionGate {
public:
    [[nodiscard]] GateDecision evaluate(const StepProposal&) const override {
        return GateDecision::Accept;
    }

    [[nodiscard]] std::string_view name() const noexcept override {
        return "accept-all";
    }
};

class RejectEvasiveGate final : public DecisionGate {
public:
    [[nodiscard]] GateDecision evaluate(const StepProposal& proposal) const override {
        return proposal.evasiveManeuver ? GateDecision::Veto : GateDecision::Accept;
    }

    [[nodiscard]] std::string_view name() const noexcept override {
        return "reject-evasive";
    }
};
}

int main() {
    try {
        auto acceptAll = std::make_shared<AcceptAllGate>();
        PlannerEngine accepted(
            {0.0, 0.0},
            {0.0, 10.0},
            {},
            acceptAll
        );
        accepted.ingestObstacleMap({{0.0, 4.0}});
        const auto acceptedPlan = accepted.calculateSafeTrajectory();
        require(acceptedPlan.reachedTarget(), "accepting gate should preserve planner behavior");
        require(acceptedPlan.gateVetoes == 0, "accepting gate should report no vetoes");

        auto rejectEvasive = std::make_shared<RejectEvasiveGate>();
        PlannerEngine gated(
            {0.0, 0.0},
            {0.0, 10.0},
            {},
            rejectEvasive
        );
        gated.ingestObstacleMap({{0.0, 4.0}});
        const auto gatedPlan = gated.calculateSafeTrajectory();

        require(!gatedPlan.reachedTarget(), "vetoed proposal must not be reported as success");
        require(
            gatedPlan.terminationReason == TerminationReason::GateVeto,
            "gate disagreement should terminate with an explicit veto state"
        );
        require(gatedPlan.gateVetoes == 1, "gate veto should be observable in telemetry");
        require(
            gatedPlan.trajectory.size() == 3,
            "vetoed motion must not be appended and must not trigger silent reselection"
        );
        require(
            gatedPlan.trajectory.back().x == 0.0 && gatedPlan.trajectory.back().y == 2.0,
            "trajectory should stop at the last independently accepted waypoint"
        );

        std::cout << "All decision-gate tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Decision-gate test failure: " << error.what() << '\n';
        return 1;
    }
}
