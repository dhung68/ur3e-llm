import pytest
from ur3_llm_control.planning import PlanError, validate_ros_domain


def test_domain_231_and_232_are_in_range():
    assert validate_ros_domain('231') == 231
    assert validate_ros_domain(232) == 232


@pytest.mark.parametrize('value',['233','-1','bad','999'])
def test_invalid_domain_rejected(value):
    with pytest.raises(PlanError): validate_ros_domain(value)
