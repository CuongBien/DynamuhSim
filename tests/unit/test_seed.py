from hashlib import sha256

from narrow_nav.core.seed import derive_seed, make_rng


def test_seed_derivation_and_repeatable_independent_streams():
    material = b"narrow_nav/v1/42/sensor/sensor.lidar"
    expected = int.from_bytes(sha256(material).digest()[:8], "big")
    assert derive_seed(42, "sensor", "sensor.lidar") == expected
    first, second = make_rng(42, "spawn"), make_rng(42, "spawn")
    independent = make_rng(42, "robot_policy")
    values = [first.random() for _ in range(4)]
    assert values == [second.random() for _ in range(4)]
    assert values != [independent.random() for _ in range(4)]
