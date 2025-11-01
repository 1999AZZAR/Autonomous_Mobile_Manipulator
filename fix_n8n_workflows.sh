#!/bin/bash

# N8N Workflow Fix Script
# This script properly imports and associates workflows with n8n

set -e

echo "🔧 N8N Workflow Fix Script"
echo "=========================="

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N8N_DATA_DIR="${PROJECT_DIR}/n8n_data"
COMPOSE_FILE="${PROJECT_DIR}/docker-compose.yml"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

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

# Stop n8n for database operations
stop_n8n() {
    print_info "Stopping n8n for database operations..."
    docker compose -f "$COMPOSE_FILE" stop n8n 2>/dev/null || true
}

# Start n8n
start_n8n() {
    print_info "Starting n8n..."
    docker compose -f "$COMPOSE_FILE" up -d n8n

    print_info "Waiting for n8n to start..."
    for i in {1..30}; do
        if curl -s http://localhost:5678 > /dev/null 2>&1; then
            print_success "n8n is ready"
            return 0
        fi
        echo -n "."
        sleep 1
    done

    print_error "n8n failed to start"
    return 1
}

# Clean existing workflows
clean_workflows() {
    print_info "Cleaning existing workflows..."
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" "DELETE FROM workflow_entity;" 2>/dev/null || true
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" "DELETE FROM shared_workflow;" 2>/dev/null || true
    print_success "Cleaned existing workflows"
}

# Import workflows using n8n CLI
import_workflows() {
    print_info "Importing workflows using n8n CLI..."

    # Import all workflow files
    docker exec $(docker ps -q -f name=n8n_container) n8n import:workflow --separate --input=/home/node/.n8n/workflows/ > /dev/null 2>&1

    print_success "Imported workflows via CLI"
}

# Associate workflows with project
associate_workflows() {
    print_info "Associating workflows with project..."

    # Get project ID
    PROJECT_ID=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT id FROM project LIMIT 1;")

    if [ -z "$PROJECT_ID" ]; then
        print_error "No project found in n8n"
        return 1
    fi

    # Associate all unshared workflows with the project
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" << EOF
INSERT OR IGNORE INTO shared_workflow (workflowId, projectId, role, createdAt, updatedAt)
SELECT id, '${PROJECT_ID}', 'workflow:owner', datetime('now'), datetime('now')
FROM workflow_entity
WHERE id NOT IN (SELECT workflowId FROM shared_workflow);
EOF

    print_success "Associated workflows with project"
}

# Verify workflows
verify_workflows() {
    print_info "Verifying workflow import..."

    WORKFLOW_COUNT=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT COUNT(*) FROM workflow_entity;")
    SHARED_COUNT=$(sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT COUNT(*) FROM shared_workflow;")

    print_success "Database contains $WORKFLOW_COUNT workflows, $SHARED_COUNT associations"

    # List robot workflows
    echo ""
    print_info "Robot Control Workflows:"
    sqlite3 "${N8N_DATA_DIR}/database.sqlite" "SELECT name FROM workflow_entity WHERE name LIKE '%Control%' OR name LIKE '%Mobile%' OR name LIKE '%Emergency%' OR name LIKE '%Calibration%' OR name LIKE '%Production%' ORDER BY name;" | while read name; do
        echo "  • $name"
    done
}

# Main execution
main() {
    cd "$PROJECT_DIR"

    echo ""
    print_info "This script will:"
    echo "  1. Stop n8n"
    echo "  2. Clean existing workflows"
    echo "  3. Import workflows using n8n CLI"
    echo "  4. Associate workflows with project"
    echo "  5. Restart n8n"
    echo ""

    read -p "Continue? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Aborted"
        exit 0
    fi

    stop_n8n
    clean_workflows
    start_n8n
    import_workflows
    associate_workflows

    stop_n8n
    start_n8n

    verify_workflows

    echo ""
    print_success "Workflow fix completed!"
    echo ""
    print_info "🌐 Access n8n at: http://localhost:5678"
    print_info "🔄 Refresh your browser if workflows don't appear immediately"
    print_info "📝 If still not visible, try: docker compose restart n8n"
}

main "$@"