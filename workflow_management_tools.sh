#!/bin/bash

# N8N Workflow Management Tools
# Comprehensive tool for importing, exporting, and cleaning N8N workflows
# Supports: import all, export all, clean all, and enhanced workflow management
#
# Environment Variables (optional):
#   N8N_DATA_DIR        - Path to n8n data directory (default: ./n8n_data)
#   WORKFLOWS_DIR       - Path to workflows directory (default: ./n8n_data/workflows)
#   EXPORT_DIR          - Path to export directory (default: ./workflow_exports)
#   BACKUP_DIR          - Path to backup directory (default: ./workflow_backups)
#   N8N_CONTAINER       - Docker container name for N8N (default: n8n_container)
#   DOCKER_COMPOSE_FILE - Path to docker-compose.yml file (default: ./docker-compose.yml)

set -e

# Configuration - can be overridden with environment variables
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N8N_DATA_DIR="${N8N_DATA_DIR:-${SCRIPT_DIR}/n8n_data}"
WORKFLOWS_DIR="${WORKFLOWS_DIR:-${N8N_DATA_DIR}/workflows}"
EXPORT_DIR="${EXPORT_DIR:-${SCRIPT_DIR}/workflow_exports}"
BACKUP_DIR="${BACKUP_DIR:-${SCRIPT_DIR}/workflow_backups}"
N8N_CONTAINER="${N8N_CONTAINER:-n8n_container}"
DOCKER_COMPOSE_FILE="${DOCKER_COMPOSE_FILE:-${SCRIPT_DIR}/docker-compose.yml}"

# Colors and formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Icons
ICON_SUCCESS="✅"
ICON_ERROR="❌"
ICON_WARNING="⚠️"
ICON_INFO="ℹ️"
ICON_ROCKET="🚀"
ICON_CLEAN="🧹"
ICON_EXPORT="📤"
ICON_IMPORT="📥"
ICON_BACKUP="💾"

# Print functions
print_header() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC} ${WHITE}N8N Workflow Management Tools${NC}                                               ${CYAN}║${NC}"
    echo -e "${CYAN}║${NC} ${PURPLE}Comprehensive workflow import/export/clean operations${NC}                       ${CYAN}║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
    echo
}

print_section() {
    echo -e "${BLUE}┌─ ${WHITE}$1${NC} ${BLUE}$(printf '%.0s─' {1..$(($(tput cols)-${#1}-5))})┐${NC}"
}

print_subsection() {
    echo -e "${YELLOW}│ ${CYAN}$1${NC} ${YELLOW}$(printf '%.0s─' {1..$(($(tput cols)-${#1}-4))})│${NC}"
}

print_footer() {
    echo -e "${BLUE}└$(printf '%.0s─' {1..$(tput cols)})┘${NC}"
    echo
}

print_success() {
    echo -e "${GREEN}${ICON_SUCCESS} $1${NC}"
}

print_error() {
    echo -e "${RED}${ICON_ERROR} $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}${ICON_WARNING} $1${NC}"
}

print_info() {
    echo -e "${BLUE}${ICON_INFO} $1${NC}"
}

print_rocket() {
    echo -e "${PURPLE}${ICON_ROCKET} $1${NC}"
}

# Check prerequisites
check_prerequisites() {
    print_section "Prerequisites Check"

    # Check if docker is running
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker is not running or not accessible"
        exit 1
    fi
    print_success "Docker is accessible"

    # Check if n8n container is running
    if ! docker ps -q -f name=${N8N_CONTAINER} | grep -q .; then
        print_error "N8N container (${N8N_CONTAINER}) is not running"
        print_info "Start N8N first with: docker compose up -d n8n"
        exit 1
    fi
    print_success "N8N container is running"

    # Check workflows directory
    if [ ! -d "$WORKFLOWS_DIR" ]; then
        print_error "Workflows directory not found: $WORKFLOWS_DIR"
        exit 1
    fi
    print_success "Workflows directory exists"

    # Create export and backup directories
    mkdir -p "$EXPORT_DIR" "$BACKUP_DIR"
    print_success "Export and backup directories ready"

    print_footer
}

# List available workflows
list_workflows() {
    print_section "Available Workflows"

    if [ ! -d "$WORKFLOWS_DIR" ]; then
        print_error "Workflows directory not found"
        return 1
    fi

    local workflow_files=("$WORKFLOWS_DIR"/*.json)
    local count=0

    if [ ${#workflow_files[@]} -eq 0 ] || [ ! -f "${workflow_files[0]}" ]; then
        print_warning "No workflow files found in $WORKFLOWS_DIR"
        return 1
    fi

    echo -e "${YELLOW}Found ${#workflow_files[@]} workflow files:${NC}"
    echo

    # Enhanced workflows
    echo -e "${CYAN}🛡️ Enhanced Individual Control Workflows:${NC}"
    for file in "$WORKFLOWS_DIR"/individual_sensor*.json "$WORKFLOWS_DIR"/individual_movement*.json "$WORKFLOWS_DIR"/individual_servo*.json "$WORKFLOWS_DIR"/individual_safety*.json "$WORKFLOWS_DIR"/individual_state*.json; do
        if [ -f "$file" ]; then
            local name=$(basename "$file" .json | sed 's/individual_//' | sed 's/_/ /g' | sed 's/\b\w/\U&/g')
            echo -e "  ${GREEN}•${NC} $name"
            ((count++))
        fi
    done

    # Combination workflows
    echo -e "${CYAN}🔄 Combination Workflows:${NC}"
    for file in "$WORKFLOWS_DIR"/robot_*.json; do
        if [ -f "$file" ]; then
            local name=$(basename "$file" .json | sed 's/robot_//' | sed 's/_/ /g' | sed 's/\b\w/\U&/g')
            echo -e "  ${GREEN}•${NC} $name"
            ((count++))
        fi
    done

    # Framework-only workflows (not fully implemented)
    echo -e "${CYAN}🔮 Framework-Only Workflows (Not Fully Implemented):${NC}"
    for file in "$WORKFLOWS_DIR"/*.json; do
        if [ -f "$file" ] && [[ "$file" =~ "Framework" || "$file" =~ "Not Implemented" ]]; then
            local name=$(basename "$file" .json | sed 's/ (Framework.*)//' | sed 's/_/ /g' | sed 's/\b\w/\U&/g')
            echo -e "  ${RED}•${NC} $name"
            ((count++))
        fi
    done

    # Legacy workflows
    echo -e "${CYAN}🔧 Legacy Workflows:${NC}"
    for file in "$WORKFLOWS_DIR"/individual_control*.json; do
        if [ -f "$file" ] && [[ ! "$file" =~ individual_control_container ]] && [[ ! "$file" =~ "Framework" ]]; then
            local name=$(basename "$file" .json | sed 's/individual_control_//' | sed 's/_/ /g' | sed 's/\b\w/\U&/g')
            echo -e "  ${YELLOW}•${NC} $name"
            ((count++))
        fi
    done

    # Simple test workflows
    echo -e "${CYAN}🧪 Test Workflows:${NC}"
    for file in "$WORKFLOWS_DIR"/robot_simple_test.json "$WORKFLOWS_DIR"/robot_test_verification.json "$WORKFLOWS_DIR"/individual_get_status.json; do
        if [ -f "$file" ] && [[ ! "$file" =~ "Framework" ]]; then
            local name=$(basename "$file" .json | sed 's/robot_//' | sed 's/individual_//' | sed 's/_/ /g' | sed 's/\b\w/\U&/g')
            echo -e "  ${BLUE}•${NC} $name"
            ((count++))
        fi
    done

    echo
    print_info "Total: $count workflow files available"
    print_footer
}

# Export workflows from N8N
export_workflows() {
    print_section "Export Workflows from N8N"

    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local export_path="$EXPORT_DIR/export_$timestamp"

    print_info "Creating export directory: $export_path"
    mkdir -p "$export_path"

    print_info "Fetching workflows from N8N..."

    # Get all workflows via API
    local workflows_json=$(docker exec ${N8N_CONTAINER} curl -s "http://localhost:5678/rest/workflows" 2>/dev/null || echo '{"data": []}')

    if ! echo "$workflows_json" | jq -e '.data' >/dev/null 2>&1; then
        print_error "Failed to fetch workflows from N8N API"
        return 1
    fi

    local workflow_count=$(echo "$workflows_json" | jq '.data | length')

    if [ "$workflow_count" -eq 0 ]; then
        print_warning "No workflows found in N8N instance"
        return 1
    fi

    print_info "Found $workflow_count workflows to export"

    # Export each workflow
    local exported=0
    for ((i=0; i<workflow_count; i++)); do
        local workflow_id=$(echo "$workflows_json" | jq -r ".data[$i].id")
        local workflow_name=$(echo "$workflows_json" | jq -r ".data[$i].name" | sed 's/[^a-zA-Z0-9._-]/_/g')

        if [ -n "$workflow_id" ] && [ "$workflow_id" != "null" ]; then
            print_info "Exporting: $workflow_name (ID: $workflow_id)"

            # Get full workflow data
            local workflow_data=$(docker exec ${N8N_CONTAINER} curl -s "http://localhost:5678/rest/workflows/$workflow_id" 2>/dev/null)

            if [ -n "$workflow_data" ]; then
                echo "$workflow_data" > "$export_path/${workflow_name}_${workflow_id}.json"
                print_success "Exported: ${workflow_name}_${workflow_id}.json"
                ((exported++))
            else
                print_warning "Failed to export workflow: $workflow_name"
            fi
        fi
    done

    # Create summary
    echo "{
  \"export_timestamp\": \"$timestamp\",
  \"export_path\": \"$export_path\",
  \"total_workflows\": $workflow_count,
  \"exported_workflows\": $exported,
  \"n8n_instance\": \"${N8N_CONTAINER}\"
}" > "$export_path/export_summary.json"

    print_success "Export completed!"
    print_info "Exported $exported workflows to: $export_path"
    print_info "Summary saved to: $export_path/export_summary.json"

    print_footer
}

# Clean workflows from N8N
clean_workflows() {
    print_section "Clean Workflows from N8N"

    print_warning "This will DELETE ALL workflows from the N8N instance!"
    read -p "Are you sure you want to continue? (type 'YES' to confirm): " -r
    echo
    if [[ ! $REPLY =~ ^YES$ ]]; then
        print_info "Operation cancelled"
        return 0
    fi

    # Stop N8N for safe database operations
    print_info "Stopping N8N for safe cleanup..."
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" stop n8n 2>/dev/null || true

    # Clean workflows from database
    print_info "Cleaning workflow tables..."
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" "DELETE FROM workflow_entity;" 2>/dev/null || true
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" "DELETE FROM shared_workflow;" 2>/dev/null || true

    print_success "Database cleaned"

    # Restart N8N
    print_info "Restarting N8N..."
    docker compose -f "${DOCKER_COMPOSE_FILE}" up -d n8n

    # Wait for N8N to be ready
    print_info "Waiting for N8N to restart..."
    local attempts=0
    while [ $attempts -lt 30 ]; do
        if docker exec ${N8N_CONTAINER} curl -s http://localhost:5678 >/dev/null 2>&1; then
            break
        fi
        sleep 1
        ((attempts++))
    done

    if [ $attempts -eq 30 ]; then
        print_error "N8N failed to restart properly"
        return 1
    fi

    # Verify cleanup
    local remaining=$(docker exec ${N8N_CONTAINER} curl -s "http://localhost:5678/rest/workflows" 2>/dev/null | jq '.data | length' 2>/dev/null || echo "0")

    if [ "$remaining" = "0" ]; then
        print_success "All workflows successfully removed from N8N"
    else
        print_warning "Some workflows may still remain ($remaining found)"
    fi

    print_footer
}

# Import workflows to N8N
import_workflows() {
    print_section "Import Workflows to N8N"

    local import_type="$1"
    local workflow_files=()

    case "$import_type" in
        "enhanced")
            print_info "Importing Enhanced Workflows..."
            # Use glob patterns to find enhanced workflow files
            for file in "$WORKFLOWS_DIR"/individual_sensor*.json \
                       "$WORKFLOWS_DIR"/individual_movement*.json \
                       "$WORKFLOWS_DIR"/individual_servo*.json \
                       "$WORKFLOWS_DIR"/individual_safety*.json \
                       "$WORKFLOWS_DIR"/individual_state*.json \
                       "$WORKFLOWS_DIR"/individual_control_container_system.json; do
                if [ -f "$file" ]; then
                    workflow_files+=("$file")
                fi
            done
            ;;
        "all")
            print_info "Importing All Workflows..."
            # Use glob expansion to find all JSON files (more reliable than process substitution)
            for file in "$WORKFLOWS_DIR"/*.json; do
                if [ -f "$file" ]; then
                    workflow_files+=("$file")
                fi
            done
            ;;
        *)
            print_error "Invalid import type. Use 'enhanced' or 'all'"
            return 1
            ;;
    esac

    local total_files=${#workflow_files[@]}
    local imported=0
    local failed=0

    if [ $total_files -eq 0 ]; then
        print_warning "No workflow files found for import type: $import_type"
        print_footer
        return 1
    fi

    print_info "Found $total_files workflow files to import"

    # Get project ID for workflow association
    print_info "Getting project ID for workflow association..."
    local project_id=$(docker cp ${N8N_CONTAINER}:/home/node/.n8n/database.sqlite /tmp/n8n_import.db 2>/dev/null && sqlite3 /tmp/n8n_import.db "SELECT id FROM project LIMIT 1;" 2>/dev/null && rm -f /tmp/n8n_import.db)

    if [ -n "$project_id" ]; then
        print_info "Using project ID: $project_id"
    else
        print_warning "Could not get project ID - workflows may not be visible in UI"
        project_id=""
    fi

    # Import each workflow
    for workflow_file in "${workflow_files[@]}"; do
        if [ -f "$workflow_file" ]; then
            local filename=$(basename "$workflow_file")
            print_info "Importing: $filename"

            # Copy to container and import with project association
            docker cp "$workflow_file" ${N8N_CONTAINER}:/home/node/.n8n/workflows/ || true
            if [ -n "$project_id" ]; then
                docker exec ${N8N_CONTAINER} n8n import:workflow --input="/home/node/.n8n/workflows/$filename" --projectId="$project_id"
            else
                docker exec ${N8N_CONTAINER} n8n import:workflow --input="/home/node/.n8n/workflows/$filename"
            fi

            if [ $? -eq 0 ]; then
                print_success "Imported: $filename"
                ((imported++))
            else
                print_error "Failed to import: $filename"
                ((failed++))
            fi

            # Cleanup
            docker exec ${N8N_CONTAINER} rm -f "/home/node/.n8n/workflows/$filename" || true
        fi
    done

    print_success "Import completed!"
    print_info "Successfully imported: $imported workflows"
    if [ $failed -gt 0 ]; then
        print_warning "Failed to import: $failed workflows"
    fi

    print_footer
}

# Show usage
show_usage() {
    print_header

    echo -e "${WHITE}Usage:${NC}"
    echo -e "  $0 <command> [options]"
    echo
    echo -e "${WHITE}Commands:${NC}"
    echo -e "  ${GREEN}list${NC}                    List all available workflows"
    echo -e "  ${GREEN}export${NC}                  Export all workflows from N8N to local files"
    echo -e "  ${GREEN}import-enhanced${NC}         Import enhanced individual control workflows"
    echo -e "  ${GREEN}import-all${NC}              Import all available workflows"
    echo -e "  ${GREEN}clean${NC}                   Clean all workflows from N8N instance"
    echo -e "  ${GREEN}backup${NC}                  Create backup of current N8N workflows"
    echo -e "  ${GREEN}status${NC}                  Show N8N instance status and workflow counts"
    echo -e "  ${GREEN}help${NC}                    Show this help message"
    echo
    echo -e "${WHITE}Examples:${NC}"
    echo -e "  $0 list                    # List available workflows"
    echo -e "  $0 export                  # Export all workflows from N8N"
    echo -e "  $0 clean                   # Remove all workflows from N8N"
    echo -e "  $0 import-enhanced         # Import enhanced control workflows"
    echo -e "  $0 import-all              # Import all available workflows"
    echo -e "  $0 status                  # Check N8N status"
    echo
    echo -e "${WHITE}Workflow Categories:${NC}"
    echo -e "  ${CYAN}🛡️ Enhanced Individual${NC}   - Comprehensive modular control systems"
    echo -e "  ${CYAN}🔄 Combination${NC}          - Multi-system automation workflows"
    echo -e "  ${CYAN}🔧 Legacy${NC}               - Basic control workflows"
    echo
    print_footer
}

# Show status
show_status() {
    print_section "N8N Instance Status"

    # Check N8N container
    if docker ps -q -f name=${N8N_CONTAINER} | grep -q .; then
        print_success "N8N container is running"
    else
        print_error "N8N container is not running"
        return 1
    fi

    # Check N8N web interface
    if docker exec ${N8N_CONTAINER} curl -s http://localhost:5678 >/dev/null 2>&1; then
        print_success "N8N web interface is accessible"
    else
        print_error "N8N web interface is not accessible"
        return 1
    fi

    # Get workflow counts
    local db_workflows=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT COUNT(*) FROM workflow_entity;" 2>/dev/null || echo "0")
    local api_workflows=$(docker exec ${N8N_CONTAINER} curl -s "http://localhost:5678/rest/workflows" 2>/dev/null | jq '.data | length' 2>/dev/null || echo "0")

    print_info "Database workflows: $db_workflows"
    print_info "API accessible workflows: $api_workflows"

    # Check local workflow files
    local local_files=$(find "$WORKFLOWS_DIR" -name "*.json" 2>/dev/null | wc -l)
    print_info "Local workflow files: $local_files"

    # Check export/backups
    local export_count=$(find "$EXPORT_DIR" -name "*.json" 2>/dev/null | wc -l)
    local backup_count=$(find "$BACKUP_DIR" -name "*.json" 2>/dev/null | wc -l)
    print_info "Exported workflows: $export_count"
    print_info "Backup files: $backup_count"

    print_footer
}

# Create backup
create_backup() {
    print_section "Create Workflow Backup"

    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local backup_path="$BACKUP_DIR/backup_$timestamp"

    print_info "Creating backup: $backup_path"
    mkdir -p "$backup_path"

    # Copy all current workflows
    if [ -d "$WORKFLOWS_DIR" ]; then
        cp -r "$WORKFLOWS_DIR"/* "$backup_path/" 2>/dev/null || true
    fi

    # Also export current N8N state if running
    if docker ps -q -f name=${N8N_CONTAINER} | grep -q .; then
        print_info "Exporting current N8N workflows..."
        export_workflows > /dev/null 2>&1
        if [ -d "$EXPORT_DIR" ]; then
            cp -r "$EXPORT_DIR"/* "$backup_path/" 2>/dev/null || true
        fi
    fi

    # Create backup summary
    echo "{
  \"backup_timestamp\": \"$timestamp\",
  \"backup_path\": \"$backup_path\",
  \"n8n_status\": \"$(docker ps -q -f name=${N8N_CONTAINER} | wc -l) container(s) running\",
  \"local_workflows\": $(find "$WORKFLOWS_DIR" -name "*.json" 2>/dev/null | wc -l),
  \"description\": \"Complete workflow backup including local files and N8N exports\"
}" > "$backup_path/backup_summary.json"

    print_success "Backup completed!"
    print_info "Backup saved to: $backup_path"
    print_info "Summary: $backup_path/backup_summary.json"

    print_footer
}

# Main function
main() {
    local command="$1"

    case "$command" in
        "list")
            print_header
            check_prerequisites
            list_workflows
            ;;
        "export")
            print_header
            check_prerequisites
            export_workflows
            ;;
        "import-enhanced")
            print_header
            check_prerequisites
            import_workflows "enhanced"
            ;;
        "import-all")
            print_header
            check_prerequisites
            import_workflows "all"
            ;;
        "clean")
            print_header
            check_prerequisites
            clean_workflows
            ;;
        "backup")
            print_header
            check_prerequisites
            create_backup
            ;;
        "status")
            print_header
            show_status
            ;;
        "help"|"-h"|"--help"|"")
            show_usage
            ;;
        *)
            print_error "Unknown command: $command"
            echo
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
