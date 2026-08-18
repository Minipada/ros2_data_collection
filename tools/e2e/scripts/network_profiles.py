# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Named network conditions (#366): one declarative place shared by run_degraded.sh and,
later, #323's limits harness, so two runs of the same profile name are comparable.

A profile is a fixed set of `tc netem` parameters applied to a shaped Destination's own
network namespace (see run_degraded.sh's header for why that's the shapeable seam under
this project's rootless container runtime): added delay, delay variation (jitter), a
loss probability, and an optional rate cap. Conditions are named and held fixed for a
run, not a ramped dimension — the PRD this implements explicitly rejects varying
conditions mid-run, since a result whose conditions moved underneath it isn't
interpretable.

`loopback` is the zero-shaping baseline: no qdisc is applied for it at all (see
`is_shaped`), so a loopback run's conditions match today's un-shaped E2E harness rather
than merely approximating it. The three degraded profiles are illustrative real-link
figures (a good office WiFi AP, a congested/bufferbloated WiFi AP, a constrained site
uplink), not measurements of any specific deployment — the PRD's own "Out of Scope"
section defers calibrating them against real hardware to a follow-up.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class NetworkProfile:
    name: str
    delay_ms: float
    jitter_ms: float
    loss_pct: float
    # kbit/s egress cap, or None for no cap.
    rate_kbit: int | None = None

    @property
    def is_shaped(self) -> bool:
        """False only for a profile that must reproduce today's un-shaped behaviour."""
        return bool(self.delay_ms or self.jitter_ms or self.loss_pct or self.rate_kbit)

    def netem_args(self) -> list[str]:
        """The `tc qdisc ... netem` argument list for this profile, or [] when
        `is_shaped` is False (nothing to apply — the caller must not add a qdisc)."""
        if not self.is_shaped:
            return []
        args = ["netem"]
        if self.delay_ms or self.jitter_ms:
            args += ["delay", f"{self.delay_ms}ms"]
            if self.jitter_ms:
                args.append(f"{self.jitter_ms}ms")
        if self.loss_pct:
            args += ["loss", f"{self.loss_pct}%"]
        if self.rate_kbit:
            args += ["rate", f"{self.rate_kbit}kbit"]
        return args


# Stated parameters, held fixed — see the module docstring for what each represents.
PROFILES: dict[str, NetworkProfile] = {
    "loopback": NetworkProfile("loopback", delay_ms=0, jitter_ms=0, loss_pct=0),
    "good-wifi": NetworkProfile("good-wifi", delay_ms=20, jitter_ms=5, loss_pct=0.1),
    "poor-wifi": NetworkProfile("poor-wifi", delay_ms=80, jitter_ms=30, loss_pct=2.0),
    "site-uplink": NetworkProfile(
        "site-uplink", delay_ms=150, jitter_ms=20, loss_pct=1.0, rate_kbit=2000
    ),
}


def resolve(name: str) -> NetworkProfile:
    """Looks up a profile by name. Raises ValueError, never silently substitutes a
    default, on an unknown name — a typo in a profile name must fail loudly rather than
    quietly running loopback under a degraded label."""
    try:
        return PROFILES[name]
    except KeyError:
        known = ", ".join(sorted(PROFILES))
        raise ValueError(f"unknown network profile '{name}' (known profiles: {known})") from None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile", help="profile name, e.g. loopback / good-wifi / poor-wifi / site-uplink"
    )
    output = parser.add_mutually_exclusive_group()
    output.add_argument(
        "--shell",
        action="store_true",
        help="print PROFILE_*=value lines, one per parameter, for `eval $(...)` in a shell script",
    )
    output.add_argument("--json", action="store_true", help="print the profile as JSON")
    output.add_argument(
        "--tc-args",
        action="store_true",
        help="print the `tc qdisc ... netem` argument list, space-separated, for a shell "
        "script to word-split onto a `tc qdisc add/change dev <if> root` command line "
        "(empty line for an unshaped profile — the caller must not add a qdisc at all)",
    )
    args = parser.parse_args()

    try:
        profile = resolve(args.profile)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(asdict(profile)))
        return 0

    if args.tc_args:
        print(" ".join(profile.netem_args()))
        return 0

    print(f"PROFILE_NAME={profile.name}")
    print(f"PROFILE_DELAY_MS={profile.delay_ms}")
    print(f"PROFILE_JITTER_MS={profile.jitter_ms}")
    print(f"PROFILE_LOSS_PCT={profile.loss_pct}")
    print(f"PROFILE_RATE_KBIT={profile.rate_kbit or 0}")
    print(f"PROFILE_IS_SHAPED={'true' if profile.is_shaped else 'false'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
