# Security Policy

## Supported branches

DC has no versioned releases yet, so there is nothing to support per version: fixes land on
branch tips.

| Branch   | What it is                                     | Security fixes                       |
| -------- | ---------------------------------------------- | ------------------------------------ |
| `jazzy`  | active development (DC 2.0, ROS 2 Jazzy)        | yes — fixes land here first          |
| `humble` | legacy line (DC 1.x, ROS 2 Humble, Fluent Bit) | high and critical severity only      |
| anything else (feature branches, forks) | not a release line             | no                                   |

## Reporting a vulnerability

**Do not open a public issue, discussion, or pull request for a security problem.**

Report it through GitHub private vulnerability reporting:
[**Report a vulnerability**](https://github.com/Minipada/ros2_data_collection/security/advisories/new)
(also reachable from the repository's *Security* tab). It is enabled on this repository and
is the preferred route — the report, the discussion, the fix, and the resulting advisory all
stay in one private place.

If GitHub is not an option, email **<d.bensoussan@proton.me>** with `SECURITY` in the subject
line. Say so in the mail if you want a PGP key and one will be sent back.

Useful in a report:

- the affected branch and commit, and which package(s) are involved
- what an attacker gains, and what access they need to get it
- reproduction steps: the DC configuration, launch file, or Destination setup that triggers it
- your own severity assessment (a CVSS vector if you have one)
- whether you intend to disclose publicly, and on what date

## What to expect

| Stage                                                | Target             |
| ---------------------------------------------------- | ------------------ |
| Acknowledgement that the report was received         | 3 business days    |
| Initial assessment: accepted or rejected, + severity | 10 business days   |
| Status update while a fix is being worked on         | every 14 days      |
| Fix on `jazzy`                                       | severity-dependent |

DC is maintained by one person with no commercial support contract behind it, so these are
targets rather than an SLA. If the acknowledgement window passes in silence, chase through the
other channel above.

## Disclosure

Disclosure is coordinated. The default embargo is 90 days from acknowledgement, or until a fix
is on `jazzy`, whichever comes first — shorter for something trivially fixed, longer if the fix
has to be coordinated upstream (ROS 2, Vector, the AWS SDK). Fixed issues are published as a
GitHub Security Advisory, with a CVE requested through GitHub when the impact warrants one.
Reporters are credited by name or handle unless they ask not to be.

## Scope

In scope: everything in this repository — the `dc_*` ROS 2 packages, the vendor packages
(`vector_vendor`, `aws_sdk_vendor`), the CI and E2E tooling under `tools/`, and the container
images built from it.

Out of scope, and better reported upstream or as a normal issue:

- ROS 2 itself and third-party dependencies (Vector, the AWS SDK, …), unless the vulnerability
  comes from how DC uses them.
- The demo configurations and the local infrastructure under `tools/infrastructure/` —
  unauthenticated Postgres, default credentials, open ports. Those are deliberately trivial so
  a demo runs on a laptop; they are not a deployment template. Documentation that presents one
  of them as production-ready *is* a bug — file it as a normal issue.
- Scanner output with no demonstrated impact on DC.
