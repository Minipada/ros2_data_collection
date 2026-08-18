# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/network_profiles.py (#366): a profile must resolve
to the parameters it declares, an unknown name must be an explicit error rather than a
silent default, and the loopback profile must be distinguishable from an absent one.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

import network_profiles


def test_a_known_profile_resolves_to_its_declared_parameters():
    profile = network_profiles.resolve("poor-wifi")
    assert profile.name == "poor-wifi"
    assert profile.delay_ms == 80
    assert profile.jitter_ms == 30
    assert profile.loss_pct == 2.0


def test_an_unknown_profile_name_is_an_explicit_error():
    with pytest.raises(ValueError, match="unknown network profile 'bogus'"):
        network_profiles.resolve("bogus")


def test_the_error_names_the_known_profiles_so_a_typo_is_diagnosable():
    with pytest.raises(ValueError, match="loopback"):
        network_profiles.resolve("bogus")


def test_loopback_is_the_only_unshaped_profile():
    assert network_profiles.resolve("loopback").is_shaped is False
    for name in ("good-wifi", "poor-wifi", "site-uplink"):
        assert network_profiles.resolve(name).is_shaped is True, name


def test_loopback_produces_no_netem_args_so_no_qdisc_is_applied():
    assert network_profiles.resolve("loopback").netem_args() == []


def test_a_degraded_profile_produces_delay_jitter_and_loss_netem_args():
    args = network_profiles.resolve("poor-wifi").netem_args()
    assert args[0] == "netem"
    assert "delay" in args and "80ms" in args and "30ms" in args
    assert "loss" in args and "2.0%" in args


def test_a_profile_with_a_rate_cap_includes_a_rate_netem_arg():
    args = network_profiles.resolve("site-uplink").netem_args()
    assert "rate" in args and "2000kbit" in args


def test_a_profile_with_only_loss_and_no_delay_omits_the_delay_clause():
    zero_delay = network_profiles.NetworkProfile(
        "test-loss-only", delay_ms=0, jitter_ms=0, loss_pct=5.0
    )
    args = zero_delay.netem_args()
    assert "delay" not in args
    assert args == ["netem", "loss", "5.0%"]


def test_two_runs_of_the_same_profile_name_are_identical():
    assert network_profiles.resolve("good-wifi") == network_profiles.resolve("good-wifi")


def test_distinct_profiles_are_distinguishable_by_name_not_just_object_identity():
    loopback = network_profiles.resolve("loopback")
    assert loopback.name == "loopback"
    assert loopback != network_profiles.NetworkProfile("not-loopback", 0, 0, 0)
