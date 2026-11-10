"""DART command disturbances must disappear before the clean grasp."""

import numpy as np

from airtruck.examples.so101_pick import noise_schedule, placements


def test_noise_schedule() -> None:
    noise = noise_schedule(375, 42, 4.0)
    np.testing.assert_array_equal(noise, noise_schedule(375, 42, 4.0))
    assert noise.shape == (375, 6)
    assert np.max(np.abs(noise)) <= 4.0
    assert np.max(np.abs(noise[:150])) > 2.0
    assert np.max(np.abs(np.diff(noise, axis=0))) < 0.5
    assert not noise[180:].any()
    assert not noise[:, 5].any()
    assert not noise[0].any()
    assert not noise_schedule(375, 42, 0).any()
    assert not np.array_equal(noise, noise_schedule(375, 43, 4.0))


def test_placement_strata() -> None:
    samples = placements(50)
    for column, bounds in enumerate(((-0.07, 0.07), (-0.28, -0.16), (-np.pi / 3, np.pi / 3))):
        np.testing.assert_allclose(np.sort(samples[:, column]), np.linspace(*bounds, 50))
