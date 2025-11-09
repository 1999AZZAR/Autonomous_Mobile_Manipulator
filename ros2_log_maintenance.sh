#!/bin/bash

# ROS2 Log Maintenance Script
# Manages log rotation, cleanup, and archiving for ROS2 logs
# Prevents disk space issues while maintaining debugging capability

set -e

# Configuration
LOG_DIR="${LOG_DIR:-./ros2_logs}"
MAX_LOG_AGE_DAYS=7
MAX_TOTAL_SIZE_MB=500
ARCHIVE_DIR="${LOG_DIR}/archive"
COMPRESS_OLD_LOGS=true

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Create directories
setup_directories() {
    log_info "Setting up log directories..."
    mkdir -p "$LOG_DIR"
    mkdir -p "$ARCHIVE_DIR"
    log_success "Directories ready"
}

# Get directory size in MB
get_dir_size_mb() {
    local dir="$1"
    if [ -d "$dir" ]; then
        du -sm "$dir" 2>/dev/null | cut -f1 || echo "0"
    else
        echo "0"
    fi
}

# Compress old log files
compress_old_logs() {
    log_info "Compressing old log files..."

    local compressed=0
    local saved_space=0

    # Find log files older than 1 day that aren't already compressed
    find "$LOG_DIR" -name "*.log" -type f -mtime +1 ! -name "*.gz" | while read -r logfile; do
        local basename=$(basename "$logfile")
        local dirname=$(dirname "$logfile")

        log_info "Compressing: $basename"

        # Get original size
        local orig_size=$(stat -f%z "$logfile" 2>/dev/null || stat -c%s "$logfile" 2>/dev/null || echo "0")

        # Compress the file
        if gzip "$logfile"; then
            local new_size=$(stat -f%z "${logfile}.gz" 2>/dev/null || stat -c%s "${logfile}.gz" 2>/dev/null || echo "0")
            local space_saved=$((orig_size - new_size))
            saved_space=$((saved_space + space_saved))
            ((compressed++))
            log_success "Compressed: $basename (${space_saved} bytes saved)"
        else
            log_warning "Failed to compress: $basename"
        fi
    done

    if [ $compressed -gt 0 ]; then
        log_success "Compressed $compressed files, saved ~$((saved_space / 1024)) KB"
    else
        log_info "No old logs to compress"
    fi
}

# Archive old logs
archive_old_logs() {
    log_info "Archiving logs older than $MAX_LOG_AGE_DAYS days..."

    local archived=0
    local archive_timestamp=$(date +"%Y%m%d_%H%M%S")
    local archive_file="$ARCHIVE_DIR/logs_archive_${archive_timestamp}.tar.gz"

    # Find files older than MAX_LOG_AGE_DAYS
    local old_files=$(find "$LOG_DIR" -name "*.log*" -type f -mtime +$MAX_LOG_AGE_DAYS)

    if [ -n "$old_files" ]; then
        log_info "Creating archive: $(basename "$archive_file")"

        # Create archive
        if echo "$old_files" | tar -czf "$archive_file" -T - 2>/dev/null; then
            # Remove archived files
            echo "$old_files" | xargs rm -f
            archived=$(echo "$old_files" | wc -l)
            log_success "Archived $archived files to $(basename "$archive_file")"
        else
            log_error "Failed to create archive"
            return 1
        fi
    else
        log_info "No old logs to archive"
    fi

    return 0
}

# Clean up old archives (keep only last 10)
cleanup_old_archives() {
    log_info "Cleaning up old archives (keeping last 10)..."

    local archive_count=$(ls -t "$ARCHIVE_DIR"/*.tar.gz 2>/dev/null | wc -l)

    if [ "$archive_count" -gt 10 ]; then
        local to_remove=$((archive_count - 10))
        log_info "Removing $to_remove old archives"

        ls -t "$ARCHIVE_DIR"/*.tar.gz 2>/dev/null | tail -n "$to_remove" | xargs rm -f
        log_success "Removed $to_remove old archives"
    else
        log_info "Archive count ($archive_count) within limits"
    fi
}

# Emergency cleanup if disk usage is too high
emergency_cleanup() {
    local current_size_mb=$(get_dir_size_mb "$LOG_DIR")

    if [ "$current_size_mb" -gt "$MAX_TOTAL_SIZE_MB" ]; then
        log_warning "Emergency cleanup triggered: ${current_size_mb}MB > ${MAX_TOTAL_SIZE_MB}MB limit"

        # Remove oldest compressed logs first
        local removed_size=0
        local removed_count=0

        # Sort by modification time and remove oldest
        find "$LOG_DIR" -name "*.log.gz" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | head -n 10 | cut -d' ' -f2- | while read -r logfile; do
            local size=$(stat -f%z "$logfile" 2>/dev/null || stat -c%s "$logfile" 2>/dev/null || echo "0")
            size=$((size / 1024 / 1024))  # Convert to MB

            if rm -f "$logfile"; then
                removed_size=$((removed_size + size))
                ((removed_count++))
                log_info "Emergency removed: $(basename "$logfile") (${size}MB)"
            fi
        done

        log_warning "Emergency cleanup complete: removed $removed_count files (${removed_size}MB)"
    fi
}

# Generate log statistics
generate_stats() {
    log_info "Generating log statistics..."

    echo "=== ROS2 Log Statistics ==="
    echo "Log Directory: $LOG_DIR"
    echo "Archive Directory: $ARCHIVE_DIR"
    echo

    # Current log directory stats
    local log_count=$(find "$LOG_DIR" -name "*.log*" -type f | wc -l)
    local log_size_mb=$(get_dir_size_mb "$LOG_DIR")
    echo "Current Logs: $log_count files, ${log_size_mb}MB"

    # Archive stats
    local archive_count=$(find "$ARCHIVE_DIR" -name "*.tar.gz" -type f | wc -l)
    local archive_size_mb=$(get_dir_size_mb "$ARCHIVE_DIR")
    echo "Archives: $archive_count files, ${archive_size_mb}MB"

    # Total size
    local total_size_mb=$((log_size_mb + archive_size_mb))
    echo "Total Size: ${total_size_mb}MB (limit: ${MAX_TOTAL_SIZE_MB}MB)"

    # Recent activity
    echo
    echo "Recent Log Activity:"
    find "$LOG_DIR" -name "*.log" -type f -mmin -60 -exec ls -la {} \; 2>/dev/null | head -5 || echo "No recent activity"

    # Error summary (last 24 hours)
    echo
    echo "Error Summary (last 24h):"
    local error_count=$(find "$LOG_DIR" -name "*.log" -type f -mtime -1 -exec grep -l "ERROR\|CRITICAL" {} \; 2>/dev/null | wc -l)
    local warning_count=$(find "$LOG_DIR" -name "*.log" -type f -mtime -1 -exec grep -l "WARNING" {} \; 2>/dev/null | wc -l)
    echo "Errors: $error_count files"
    echo "Warnings: $warning_count files"
}

# Main maintenance function
main() {
    log_info "Starting ROS2 Log Maintenance"
    echo "================================="

    setup_directories

    # Generate initial stats
    generate_stats
    echo

    # Emergency cleanup if needed
    emergency_cleanup

    # Regular maintenance
    if [ "$COMPRESS_OLD_LOGS" = true ]; then
        compress_old_logs
        echo
    fi

    archive_old_logs
    echo

    cleanup_old_archives
    echo

    # Final stats
    log_info "Maintenance complete"
    generate_stats

    log_success "ROS2 Log Maintenance finished successfully"
}

# Run main function
main "$@"
