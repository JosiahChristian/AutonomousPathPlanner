#ifndef DECISION_GATE_HPP
#define DECISION_GATE_HPP

#include <string_view>

struct Position;

enum class GateDecision {
    Accept,
    Veto
};

struct StepProposal {
    Position from;
    Position to;
    Position target;
    bool evasiveManeuver{false};
};

class DecisionGate {
public:
    virtual ~DecisionGate() = default;

    [[nodiscard]] virtual GateDecision evaluate(const StepProposal& proposal) const = 0;
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

#endif
