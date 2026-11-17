import pytest
from robot_validate.model import Check, Result, load_config


def test_missing_required_evidence_does_not_pass():
    result = Result("demo", "local", 1)
    result.checks = [Check("map", "mapping", "pass"), Check("ate", "mapping", "insufficient")]
    assert result.status == "insufficient"
    assert result.as_dict()["sections"]["navigation"] == "insufficient"
    result.checks.append(Check("gap", "mapping", "fail"))
    assert result.status == "fail"


def test_bad_threshold_is_rejected(tmp_path):
    config = tmp_path / "bad.yaml"
    config.write_text("thresholds:\n  min_pair_coverage: 2\n")
    with pytest.raises(ValueError, match="fraction"):
        load_config(config)
