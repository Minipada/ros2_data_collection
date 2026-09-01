# Split deployment topology: the Shipper and the Uploader as separable processes

ADR-0001 put the Shipper on localhost, supervised by DC's bringup, because that was
correct for one robot and did not need to be anything else at the time. Epic #440 asks DC
to run as a fleet: robots with no internet access, forwarding through an edge Vector
aggregator, needing per-component restarts, resource limits, and credentials that a single
process tree cannot give. This ADR records where the decomposition line goes and why it
stops there.

## Decision

The robot decomposes into three processes — the ROS stack (with the Bridge), the Shipper,
and the Uploader — and no further. Whether those three run as one process tree (ADR-0001's
original native mode), three containers on one machine (#447), or spread across robot,
edge, and hub tiers (#440's target architecture) is a deployment-time choice, not a code
change: the Bridge's unmanaged-shipper mode (#444) and the Uploader's extraction into its
own process (ADR-0014) are what let the same binaries run in every shape.

**Why the decomposition stops at the Shipper and the Uploader, and does not extend to ROS
nodes.** DDS is the wrong protocol to run over anything but a robot's own local network —
it assumes multicast discovery and low-latency links that neither an edge tunnel nor a
security-conscious deployment can offer (epic #440's user stories 21/22: DDS stays on the
robot, no inbound connection reaches it). The Shipper and the Uploader are the two
components that already speak something else — the shipper ingest protocol and a plain
object-storage API, respectively — so they are the only components that can cross a
process, container, or host boundary without inventing a new transport for the purpose.
Every ROS node, including the Bridge, stays together in one container on one machine per
robot.

## Rejected alternative: routing File bytes through the Shipper or a database

Considered and rejected: instead of the Uploader talking to object storage directly, put
File bytes on the same path as Records — base64 them into a Record field, or write them
into a database column. Rejected because:

- Base64 inflates payload size by roughly a third, for images and videos that are already
  the largest artifacts DC moves.
- Multi-megabyte Records force the Shipper's buffer, and any relational Destination's row
  storage (PostgreSQL TOAST), to handle a size class neither is designed for — at the cost
  of every other Record sharing that same path.
- On a fleet robot's constrained uplink, the bytes would cross the network twice: once to
  the Destination, and once because neither the Shipper nor a database offers a way to
  skip re-sending a field that already made it through.

DC keeps the claim-check pattern instead (ADR-0005): the Record carries a reference, the
Uploader moves the bytes out of band.

## Consequences

- Managed mode (one supervised process tree, ADR-0001's original design) and unmanaged
  mode (#444, this ADR) are both first-class; native install and simulation do not need
  containers to stay correct.
- The Shipper's own buffer and the Bridge's upload intent queue are separate volumes with
  separate owners — a container boundary makes visible, as two mount points, what was
  already two independent pieces of on-disk state internally.
- `compose.split.yaml` (#447) is the concrete rendering of this decision for one machine;
  Podman Quadlet and Kubernetes manifests for the fleet tiers describe the same topology
  differently, not a different decision.
- ADR-0001 is amended alongside this ADR: "runs on localhost, supervised by DC's bringup"
  is no longer true of every deployment.
