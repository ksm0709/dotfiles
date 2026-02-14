#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_compose="${script_dir}/compose.yaml"

if [ ! -f "${source_compose}" ]; then
  echo "Error: compose.yaml not found at ${source_compose}"
  exit 1
fi

default_work_dir="${HOME}/open-notebook"
read -r -p "Open Notebook working directory [${default_work_dir}]: " input_work_dir
work_dir="${input_work_dir:-${default_work_dir}}"

if [[ "${work_dir}" == "~" ]]; then
  work_dir="${HOME}"
elif [[ "${work_dir}" == ~/* ]]; then
  work_dir="${HOME}/${work_dir#~/}"
fi

if [[ "${work_dir}" != /* ]]; then
  work_dir="$(pwd)/${work_dir}"
fi

mkdir -p "${work_dir}"

destination_compose="${work_dir}/compose.yaml"
if [ -f "${destination_compose}" ]; then
  backup_path="${destination_compose}.bak.$(date +%Y%m%d%H%M%S)"
  cp "${destination_compose}" "${backup_path}"
  echo "Backed up existing compose.yaml to ${backup_path}"
fi

cp "${source_compose}" "${destination_compose}"

sed -i \
  -e 's|\$HOME/notebook_data|./notebook_data|g' \
  -e 's|\$HOME/surreal_data|./surreal_data|g' \
  -e 's|\$HOME/secrets/password.txt|./secrets/password.txt|g' \
  -e 's|\$HOME/secrets/encryption_key.txt|./secrets/encryption_key.txt|g' \
  "${destination_compose}"

mkdir -p \
  "${work_dir}/secrets" \
  "${work_dir}/notebook_data" \
  "${work_dir}/surreal_data"

password_file="${work_dir}/secrets/password.txt"
if [ ! -s "${password_file}" ]; then
  printf 'change-me\n' > "${password_file}"
fi

encryption_key_file="${work_dir}/secrets/encryption_key.txt"
if [ ! -s "${encryption_key_file}" ]; then
  head -c 48 /dev/urandom | base64 | tr -d '\n' > "${encryption_key_file}"
  printf '\n' >> "${encryption_key_file}"
fi

chmod 600 "${password_file}" "${encryption_key_file}"

echo ""
echo "Open Notebook workspace initialized at: ${work_dir}"
echo "Created/updated:"
echo "  - ${destination_compose}"
echo "  - ${work_dir}/secrets"
echo "  - ${work_dir}/notebook_data"
echo "  - ${work_dir}/surreal_data"
echo ""
echo "Next:"
echo "  cd \"${work_dir}\""
echo "  docker compose up -d"
