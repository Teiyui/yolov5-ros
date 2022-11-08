COMMANDS="
bandit -c pyproject.toml -r ./
black --check ./
flake8 -v ./
isort --check ./
pyright --warnings ./
"

SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd "${SCRIPT_DIR}/../"

# ------------------------------------------------------------------------------------------------------------------
#
#   Main
#
# ------------------------------------------------------------------------------------------------------------------
IFS=$'\n'
for command in ${COMMANDS}; do
  eval "${command}"
  status="${?}"
  if [ ${status} -ne 0 ]; then
    echo "Error in ${command}"
    exit 1
  fi
done

echo "No error code."