"""Offline test of arm_carry.py's choice among the bridge's drop points, with sesame_pickup.py replaced by a stub.

    .venv/bin/python test_arm_carry.py            (or: pytest test_arm_carry.py)
"""
import arm_carry


def run(plannable, executes=True, refusal="no reachable drop point near the pick"):
    calls = []

    def stub(drop, extra, dry):
        calls.append((tuple(drop), dry))
        if dry:
            return (tuple(drop) in plannable), "" if tuple(drop) in plannable else refusal
        return executes, "" if executes else "the arm stalled during the lift"

    arm_carry.pickup = stub
    answer = arm_carry.carry({"id": 4, "drops": [[50, 20], [56, 24], [60, 30]]}, [])
    return answer, calls


def test_carry_choice():
    answer, calls = run({(56, 24), (60, 30)})
    assert answer == {"id": 4, "ok": True, "reason": "", "drop": [56, 24]}, answer
    assert calls == [((50, 20), True), ((56, 24), True), ((56, 24), False)], calls
    print("  PASS the first drop point that plans is the one that is run, and only that one")

    answer, calls = run(set())
    assert answer["ok"] is False and "no reachable drop point" in answer["reason"] and len(calls) == 3, (answer, calls)
    print("  PASS no drop point plans: refused with sesame_pickup.py's reason, the arm never moves")

    answer, calls = run(set(), refusal="the grip point is 41 cm from the arm's base")
    assert answer["ok"] is False and len(calls) == 1, (answer, calls)
    print("  PASS the Sesame itself is out of reach: refused after the first attempt")

    answer, calls = run({(50, 20)}, executes=False)
    assert answer["ok"] is False and "stalled" in answer["reason"], answer
    print("  PASS a carry that fails while running is reported as refused")


if __name__ == "__main__":
    test_carry_choice()
    print("RESULT: PASS")
