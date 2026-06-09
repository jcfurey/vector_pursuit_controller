#!/usr/bin/env bash
# Run a command with a CA bundle that merges the system store with any certs
# mounted at /hostcerts (a build-only mount; see docker/Dockerfile). The
# merged bundle exists only for the duration of the command and is removed
# within the same RUN step, so extra CAs never persist in an image layer.
# With no /hostcerts certs present this degrades to the plain system bundle.
set -euo pipefail
bundle="$(mktemp)"
trap 'rm -f "$bundle"' EXIT
# awk 1 (not cat): certs without a trailing newline would otherwise glue
# adjacent PEM blocks together and corrupt the bundle.
awk 1 /etc/ssl/certs/ca-certificates.crt > "$bundle"
awk 1 /hostcerts/*.crt >> "$bundle" 2>/dev/null || true
SSL_CERT_FILE="$bundle" REQUESTS_CA_BUNDLE="$bundle" CURL_CA_BUNDLE="$bundle" \
  GIT_SSL_CAINFO="$bundle" "$@"
