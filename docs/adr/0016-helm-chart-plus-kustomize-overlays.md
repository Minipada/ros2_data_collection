# Helm chart for the robot tier, Kustomize overlays per site/robot

`deploy/robot/` (#450) renders the robot tier's fixed topology three ways (Compose,
Quadlet, Kubernetes) — one shape, identical on every run. `tools/kind/` (#452) faced the
same question for its CI harness and rejected Helm there: that harness deploys one fixed
topology every run, nothing to parameterize, so a chart would template a variance that
doesn't exist (`tools/kind/README.md`'s "Why not Helm"). That reasoning does not carry
over to `deploy/robot/`'s actual deployable manifests: a fleet's robots genuinely differ
— edge aggregator address, robot identity, resource limits, image tags, credentials — and
`kubernetes/robot-pod.yaml` has no way to express that difference except hand-editing a
copy per site, the same problem `tools/kind/kubernetes/robot-a.yaml` already shows in
miniature (a second, hand-forked copy of the base Pod for one test site).

## Decision

Two tools, two different jobs, composed in one command:

- **Helm** owns the values surface: whatever genuinely varies per deployment and is
  worth typing, defaulting, and validating as a named field — robot identity
  (`robot.name`), image repository/tag per container, per-container resource
  requests/limits, the edge aggregator's address (`edge.vectorHost`/`vectorPort`), and
  the Uploader's S3 credentials/endpoint. `deploy/robot/helm/dc-robot/` is this chart.
  Its defaults reproduce `kubernetes/robot-pod.yaml` + `params/robot_params.yaml` field
  for field (verified: `helm template` with no overrides passes the same `kubeconform`
  check that file does) — the chart is a parameterized version of the existing
  reference, not a new design, and that file remains the runtime-free/k3d/kind reference
  it already was.
- **Kustomize** owns what a chart's values don't, and shouldn't, cover: per-site
  placement and cluster-specific concerns that have nothing to do with the
  application's own configuration — which namespace a site's release lands in, which
  node it's scheduled to, any raw patch a site needs that doesn't rise to the level of
  a chart value. `deploy/robot/helm/overlays/site-a/` is the reference overlay: it sets
  `namespace: dc-robot-site-a`, supplies `values-site-a.yaml` (robot identity + edge
  address — the chart's own surface), and patches in a `nodeSelector` pinning the Pod
  to that site's labeled edge-adjacent node — deliberately not a chart value, because
  which node a robot lands on is a per-cluster scheduling fact the chart has no business
  knowing about.

The two compose through kustomize's native Helm chart inflator (`helmGlobals.chartHome` +
`helmCharts:`, `--enable-helm`), not a two-step `helm template | kubectl apply -k -`
pipeline: one command (`kubectl kustomize --enable-helm --load-restrictor
LoadRestrictionsNone deploy/robot/helm/overlays/site-a`) renders the chart and applies
the overlay's namespace/patches to the result, matching the single-command shape
`deploy/robot/k3d/kustomization.yaml` already established for piping `kubectl kustomize`
into `kubectl apply -f -`. `--load-restrictor LoadRestrictionsNone` is required for the
same reason it already is there: the overlay's base (the chart, via `chartHome: ../..`)
lives outside the overlay's own directory tree.

A site with N robots installs N releases — one per robot, each with its own
`values-<robot>.yaml` and, where needed, its own overlay directory — never one release
templating N robots internally; `robot.name` (and, by convention, the release's
namespace) is the per-robot identity, matching how `tools/kind/kubernetes/robot-a.yaml`
already names one Pod per test site rather than templating a list.

## Rejected alternatives

**Kustomize alone, patching `kubernetes/robot-pod.yaml` per site (no Helm).** This is
what `tools/kind/` already does, correctly, for its own one-fixed-topology problem.
`deploy/robot/`'s real fleet has a values surface shared and validated the same way
across every site — image tags bumped in one place per release, resource limits typed
as actual Kubernetes `resources` stanzas, credentials with a documented shape — that
raw per-site strategic-merge patches would reduplicate at every overlay instead of
declaring once. Kustomize's own patches are the right tool for one-off, structural
differences (a `nodeSelector`, an extra label); they are the wrong tool for a values
contract every site fills in.

**Helm alone, one `values-<site>.yaml` per site, no Kustomize.** Considered, since Helm
values could technically carry a `nodeSelector` or namespace override too. Rejected
because it pushes every future site-specific concern into the chart's own values schema
regardless of whether it belongs there, growing `values.yaml` into a dumping ground and
coupling unrelated cluster-placement changes to chart version bumps. Kustomize overlays
keep that class of change scoped to the site's own directory, reviewable independently
of the chart.

**Per-site values vendored as Helm subcharts or an umbrella chart.** Rejected: it
couples a site's own inventory (and, worse, tempts committing its credentials) into the
chart's own repository structure. Overlay directories under `helm/overlays/` keep
site inventory as plain files a site's own GitOps tooling can manage independently of
chart releases, with credentials passed via an uncommitted `-f`/`--set` file rather than
living in the chart.

## Consequences

- Additive only: `compose.yaml` and `quadlet/*.container` are untouched, and
  `kubernetes/robot-pod.yaml` remains the runtime-free/k3d/kind reference — this ADR
  adds a parameterized *rendering path* for real Kubernetes deployments, not a
  replacement of any of the three.
- `deploy/robot/helm/dc-robot/templates/configmap-robot-params.yaml` replaces
  `robot-pod.yaml`'s `hostPath` mount for `robot_params.yaml` with a ConfigMap — the
  real-cluster mechanism that file's own header already called for
  ("a Pod-scoped Secret/ConfigMap object is the real-cluster mechanism ... kustomize or
  a site's own GitOps tool"). `robot-pod.yaml` itself keeps the `hostPath`, since that
  is what makes `podman kube play`/k3d/kind runnable with no cluster-side object to
  create first.
- Values that hold real credentials (`uploader.s3.accessKeyId`/`secretAccessKey`) ship
  with the same literal placeholders `params/robot_params.yaml`/`robot-pod.yaml` already
  use ("changeme"), for the same reason: runnable as shipped, not a design invitation to
  commit real ones — a real deployment overrides them from an uncommitted values file,
  never a committed one.
- `tools/kind/README.md`'s "Why not Helm" stays correct for that harness: it is a
  different problem (one fixed topology, nothing to parameterize) from the one this ADR
  answers.
