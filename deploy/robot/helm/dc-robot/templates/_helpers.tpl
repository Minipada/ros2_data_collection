# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Shared labels for the chart's two objects (Pod, ConfigMap) — a leading underscore
# keeps Helm from rendering this file as a manifest of its own.
{{- define "dc-robot.labels" -}}
app.kubernetes.io/part-of: dc-robot-tier
app.kubernetes.io/name: dc-robot
app.kubernetes.io/instance: {{ .Values.robot.name }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end -}}
