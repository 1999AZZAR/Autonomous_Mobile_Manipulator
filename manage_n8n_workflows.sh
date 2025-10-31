#!/bin/bash

# N8N Workflow Management Script
# This script manages n8n workflows by manipulating the database directly

set -e

# Configuration
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKFLOWS_DIR="${PROJECT_DIR}/n8n_data/workflows"
N8N_DATA_DIR="${PROJECT_DIR}/n8n_data"
COMPOSE_FILE="${PROJECT_DIR}/docker-compose.yml"

WORKFLOW_FILES=(
    "individual_control_omni_wheels.json"
    "individual_control_picker_system.json"
    "individual_control_container_system.json"
    "individual_control_hardware_controls.json"
    "robot_mobile_pick_place.json"
    "robot_inspection_patrol.json"
    "robot_material_transport.json"
    "robot_search_retrieve.json"
    "robot_emergency_response.json"
    "robot_system_calibration.json"
    "robot_production_line.json"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Functions
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Check if n8n is running
check_n8n_running() {
    if docker ps | grep -q n8n_container; then
        print_warning "n8n container is running. Stopping it for database access..."
        docker compose -f "$COMPOSE_FILE" stop n8n
        sleep 2
    fi
}

# Start n8n service
start_n8n() {
    print_info "Starting n8n service..."
    docker compose -f "$COMPOSE_FILE" up -d n8n
    sleep 3

    # Wait for n8n to be ready
    local max_attempts=30
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        if curl -s http://localhost:5678 > /dev/null 2>&1; then
            print_success "n8n is ready"
            return 0
        fi
        echo -n "."
        sleep 1
        ((attempt++))
    done

    print_error "n8n failed to start within ${max_attempts} seconds"
    return 1
}

# Clean database workflows
clean_database_workflows() {
    print_info "Cleaning existing workflows from database..."

    if [ ! -f "${N8N_DATA_DIR}/database.sqlite" ]; then
        print_info "No database file found - nothing to clean"
        return 0
    fi

    # Use sqlite3 to delete all workflows
    if command -v sqlite3 &> /dev/null; then
        local workflow_count
        workflow_count=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT COUNT(*) FROM workflow_entity;" 2>/dev/null || echo "0")

        if [ "$workflow_count" -gt 0 ]; then
            sqlite3 "${N8N_DATA_DIR}/database.sqlite" "DELETE FROM workflow_entity;" 2>/dev/null
            print_success "Cleaned $workflow_count workflows from database"
        else
            print_info "No workflows found in database"
        fi
    else
        print_warning "sqlite3 not available - cannot clean database directly"
        print_warning "You may need to manually clean workflows through the n8n web interface"
    fi
}

# Import workflow to database
import_workflow_to_db() {
    local workflow_file="$1"
    local file_path="${WORKFLOWS_DIR}/${workflow_file}"

    if [ ! -f "$file_path" ]; then
        print_error "Workflow file not found: $file_path"
        return 1
    fi

    if ! command -v sqlite3 &> /dev/null; then
        print_warning "sqlite3 not available - skipping database import"
        return 1
    fi

    print_info "Importing workflow to database: $workflow_file"

    # Read the JSON file
    local json_content
    json_content=$(cat "$file_path")

    # Extract workflow data
    local workflow_name
    workflow_name=$(echo "$json_content" | jq -r '.name // "Unknown Workflow"' 2>/dev/null || echo "Unknown Workflow")

    local active
    active=$(echo "$json_content" | jq -r '.active // false' 2>/dev/null || echo "false")
    active=$([ "$active" = "true" ] && echo "1" || echo "0")

    # Generate a UUID-like ID
    local workflow_id
    workflow_id=$(uuidgen 2>/dev/null || echo "$(date +%s%3N)-$(shuf -i 1000-9999 -n 1)")

    # Extract and escape JSON fields
    local nodes_json connections_json settings_json static_data_json pin_data_json meta_json

    nodes_json=$(echo "$json_content" | jq -c '.nodes // []' 2>/dev/null | sed "s/'/''/g")
    connections_json=$(echo "$json_content" | jq -c '.connections // {}' 2>/dev/null | sed "s/'/''/g")
    settings_json=$(echo "$json_content" | jq -c '.settings // {}' 2>/dev/null | sed "s/'/''/g")
    static_data_json=$(echo "$json_content" | jq -c '.staticData // null' 2>/dev/null | sed "s/'/''/g")
    pin_data_json=$(echo "$json_content" | jq -c '.pinData // {}' 2>/dev/null | sed "s/'/''/g")
    meta_json=$(echo "$json_content" | jq -c '.meta // null' 2>/dev/null | sed "s/'/''/g")
    trigger_count=$(echo "$json_content" | jq -r '.triggerCount // 0' 2>/dev/null || echo "0")

    # Insert workflow into database
    local sql_error
    sql_error=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" << EOF 2>&1
INSERT INTO workflow_entity (id, name, active, nodes, connections, settings, staticData, pinData, triggerCount, meta, createdAt, updatedAt)
VALUES (
    '${workflow_id}',
    '${workflow_name}',
    ${active},
    '${nodes_json}',
    '${connections_json}',
    '${settings_json}',
    '${static_data_json}',
    '${pin_data_json}',
    ${trigger_count},
    '${meta_json}',
    datetime('now'),
    datetime('now')
);
EOF
)

    if [ $? -eq 0 ]; then
        print_success "Imported workflow: $workflow_file"
        return 0
    else
        print_error "Failed to import workflow: $workflow_file"
        print_error "SQL Error: $sql_error"
        return 1
    fi
}

# Import all workflows to database
import_all_workflows_to_db() {
    print_info "Importing all workflows to database..."

    if ! command -v sqlite3 &> /dev/null; then
        print_error "sqlite3 is required for database operations"
        print_error "Please install sqlite3: sudo apt install sqlite3"
        return 1
    fi

    local success_count=0
    local total_count=${#WORKFLOW_FILES[@]}

    # Temporarily disable exit on error for individual workflow imports
    set +e

    for workflow_file in "${WORKFLOW_FILES[@]}"; do
        if import_workflow_to_db "$workflow_file"; then
            ((success_count++))
        else
            print_warning "Continuing with next workflow..."
        fi
    done

    # Re-enable exit on error
    set -e

    print_success "Imported $success_count/$total_count workflows to database"
    return 0  # Always return success to continue script
}

# Alternative method: Use n8n CLI if available
use_n8n_cli_method() {
    print_info "Attempting to use n8n CLI method..."

    # Check if we can run n8n commands in the container
    if docker run --rm -v "${N8N_DATA_DIR}:/home/node/.n8n" docker.io/n8nio/n8n:latest n8n --help > /dev/null 2>&1; then
        print_info "Using n8n CLI for workflow management"

        # This would require more complex CLI commands
        # For now, we'll stick with the database method
        return 1
    else
        print_warning "n8n CLI not accessible"
        return 1
    fi
}

# Verify imports
verify_imports() {
    print_info "Verifying workflow imports..."

    if [ ! -f "${N8N_DATA_DIR}/database.sqlite" ]; then
        print_warning "Database file not found"
        return 1
    fi

    if ! command -v sqlite3 &> /dev/null; then
        print_warning "Cannot verify without sqlite3"
        return 1
    fi

    local workflow_count
    workflow_count=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT COUNT(*) FROM workflow_entity;" 2>/dev/null || echo "0")

    if [ "$workflow_count" -eq "${#WORKFLOW_FILES[@]}" ]; then
        print_success "All workflows imported successfully ($workflow_count workflows)"
    else
        print_warning "Expected ${#WORKFLOW_FILES[@]} workflows, found $workflow_count"
    fi
}

# Main execution
main() {
    echo "========================================"
    echo "    N8N Workflow Management"
    echo "========================================"
    echo ""

    # Check prerequisites
    if ! command -v docker &> /dev/null; then
        print_error "docker is required but not installed"
        exit 1
    fi

    if ! command -v jq &> /dev/null; then
        print_error "jq is required for JSON processing"
        print_error "Please install jq: sudo apt install jq"
        exit 1
    fi

    # Change to project directory
    cd "$PROJECT_DIR"

    # Check if n8n is running and stop it
    check_n8n_running

    # Clean existing workflows
    clean_database_workflows

    # Import new workflows
    import_all_workflows_to_db

    # Start n8n service
    if start_n8n; then
        echo ""
        verify_imports

        echo ""
        print_success "Workflow management completed!"
        print_info "Access n8n at: http://localhost:5678"
        print_info "You can now view and activate the imported workflows"
    else
        print_error "Failed to start n8n service"
        exit 1
    fi
}

# Run main function
main "$@"
