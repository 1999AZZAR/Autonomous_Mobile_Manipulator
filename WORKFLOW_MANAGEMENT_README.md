# 🤖 N8N Workflow Management Tools

A comprehensive command-line tool for managing N8N workflows with advanced import, export, backup, and cleanup capabilities.

## 🚀 Features

### **Core Operations**
- **📥 Import Workflows**: Import enhanced individual control workflows or all available workflows
- **📤 Export Workflows**: Export all workflows from running N8N instance to local files
- **🧹 Clean Workflows**: Safely remove all workflows from N8N instance
- **💾 Backup Workflows**: Create timestamped backups of workflows and N8N state
- **📊 Status Monitoring**: Check N8N instance status and workflow counts
- **📋 List Workflows**: Display all available workflow files with categorization

### **Enhanced Workflow Categories**
- **🛡️ Sensor Monitoring**: Ultrasonic, IR proximity, line following with safety integration
- **🎯 Precise Movement**: Distance-based navigation and angle-based rotation control
- **🔧 Advanced Servo Control**: Safety-limited servo positioning with verification
- **⚙️ Servo Sequences**: Pre-programmed manipulation patterns for complex operations
- **🛡️ Safety Systems**: Multi-level emergency response and continuous monitoring
- **📊 State Management**: Comprehensive robot state tracking and health monitoring
- **📦 Container Framework**: Ready-for-implementation container management system

## 📋 Usage

### **Basic Commands**

```bash
# Show help and available commands
./workflow_management_tools.sh help

# List all available workflows
./workflow_management_tools.sh list

# Check N8N status and workflow counts
./workflow_management_tools.sh status
```

### **Import Operations**

```bash
# Import enhanced individual control workflows (recommended)
# Dynamically discovers and imports all enhanced workflow files
./workflow_management_tools.sh import-enhanced

# Import all available workflows
# Dynamically discovers and imports all workflow files in the directory
./workflow_management_tools.sh import-all
```

#### **Dynamic File Discovery**
- **No hardcoded file lists** - Automatically finds workflow files based on naming patterns
- **Enhanced workflows**: All files matching `individual_sensor*`, `individual_movement*`, `individual_servo*`, `individual_safety*`, `individual_state*`, plus container system
- **All workflows**: Every `.json` file in the workflows directory
- **Future-proof**: New workflow files are automatically included without code changes

### **Export & Backup**

```bash
# Export all workflows from N8N to local files
./workflow_management_tools.sh export

# Create comprehensive backup of workflows and N8N state
./workflow_management_tools.sh backup
```

### **Maintenance**

```bash
# Clean all workflows from N8N instance (with confirmation)
./workflow_management_tools.sh clean
```

## 🔄 Workflow Management Workflow

### **For Development & Testing**
1. **Backup current state**: `./workflow_management_tools.sh backup`
2. **Clean N8N instance**: `./workflow_management_tools.sh clean`
3. **Import fresh workflows**: `./workflow_management_tools.sh import-enhanced`

### **For Production Deployment**
1. **Export current workflows**: `./workflow_management_tools.sh export`
2. **Clean for fresh start**: `./workflow_management_tools.sh clean`
3. **Import production workflows**: `./workflow_management_tools.sh import-all`

### **For Maintenance**
1. **Check status**: `./workflow_management_tools.sh status`
2. **Create backup**: `./workflow_management_tools.sh backup`
3. **List available workflows**: `./workflow_management_tools.sh list`

## 📁 Directory Structure

```
project/
├── workflow_management_tools.sh          # Main management script
├── n8n_data/
│   ├── workflows/                         # Source workflow files
│   └── database.sqlite                    # N8N database
├── workflow_exports/                      # Exported workflows
│   └── export_20241101_143000/           # Timestamped exports
│       ├── workflow_name_id.json
│       └── export_summary.json
└── workflow_backups/                     # Backup archives
    └── backup_20241101_143000/           # Timestamped backups
```

## 🛡️ Safety Features

### **Confirmation Prompts**
- **Clean operations** require explicit "YES" confirmation
- **Status checks** before destructive operations
- **Automatic backups** before major changes

### **Error Handling**
- **Graceful failures** with detailed error messages
- **Partial operation recovery** when possible
- **Automatic cleanup** of temporary files

### **Prerequisites Validation**
- **Docker accessibility** check
- **N8N container status** verification
- **Directory permissions** validation
- **Network connectivity** testing

## 📊 Output Examples

### **Status Check**
```
╔══════════════════════════════════════════════════════════════════════════════╗
║ N8N Workflow Management Tools                                               ║
║ Comprehensive workflow import/export/clean operations                       ║
╚══════════════════════════════════════════════════════════════════════════════╝

┌─ N8N Instance Status ──────────────────────────────────────────────────────┐
✅ N8N container is running
✅ N8N web interface is accessible
ℹ Database workflows: 24
ℹ API accessible workflows: 24
ℹ Local workflow files: 33
ℹ Exported workflows: 12
ℹ Backup files: 8
└────────────────────────────────────────────────────────────────────────────┘
```

### **Import Operation**
```
╔══════════════════════════════════════════════════════════════════════════════╗
║ N8N Workflow Management Tools                                               ║
║ Comprehensive workflow import/export/clean operations                       ║
╚══════════════════════════════════════════════════════════════════════════════╝

┌─ Prerequisites Check ──────────────────────────────────────────────────────┐
✅ Docker is accessible
✅ N8N container is running
✅ Workflows directory exists
✅ Export and backup directories ready
└────────────────────────────────────────────────────────────────────────────┘

┌─ Import Workflows to N8N ──────────────────────────────────────────────────┐
ℹ Importing Enhanced Workflows...
ℹ Found 10 workflow files to import
📤 Importing: individual_sensor_ultrasonic_monitoring.json
✅ Imported: individual_sensor_ultrasonic_monitoring.json
✅ Imported: individual_sensor_ir_proximity.json
...
ℹ Associating workflows with project...
✅ Workflows associated with project
✅ Import completed!
ℹ Successfully imported: 10 workflows
└────────────────────────────────────────────────────────────────────────────┘
```

## 🔧 Technical Details

### **Dependencies**
- **Docker**: For N8N container management
- **SQLite3**: For database operations
- **curl**: For API communication
- **jq**: For JSON processing (optional, for enhanced output)

### **Dynamic File Discovery Architecture**

#### **File Pattern Matching**
The script uses shell globbing patterns to dynamically discover workflow files:

```bash
# Enhanced workflows pattern matching
"$WORKFLOWS_DIR"/individual_sensor*.json     # Ultrasonic, IR, Line sensors
"$WORKFLOWS_DIR"/individual_movement*.json   # Distance, angle controls
"$WORKFLOWS_DIR"/individual_servo*.json      # Servo advanced & sequence
"$WORKFLOWS_DIR"/individual_safety*.json     # Safety & error handling
"$WORKFLOWS_DIR"/individual_state*.json      # State management systems
# Plus: individual_control_container_system.json (if exists)
```

#### **All Workflows Discovery**
```bash
# Simple globbing for all JSON files
"$WORKFLOWS_DIR"/*.json
```

#### **Benefits of Dynamic Discovery**
- **Zero maintenance**: No need to update script when adding new workflows
- **Pattern-based**: Logical grouping based on file naming conventions
- **Robust**: Handles missing files gracefully
- **Extensible**: Easy to add new workflow categories

### **Environment Requirements**
- **N8N container** named `n8n_container` must be running
- **Docker Compose** setup in project root
- **Workflow files** in `n8n_data/workflows/` directory
- **Write permissions** for export and backup directories

### **API Endpoints Used**
- `GET /rest/workflows` - List all workflows
- `GET /rest/workflows/{id}` - Get specific workflow
- Database direct access for advanced operations

## 🚨 Important Notes

### **Backup First**
Always create backups before performing clean or major import operations:
```bash
./workflow_management_tools.sh backup
```

### **N8N Restart Required**
Some operations may require N8N restart for changes to take effect. The script handles this automatically.

### **Large Imports**
For large workflow imports, the script provides progress feedback and handles partial failures gracefully.

### **Permissions**
Ensure the script has write permissions to export and backup directories.

## 🎯 Best Practices

### **Development Workflow**
1. **List available workflows**: Check what's available
2. **Backup current state**: Always backup before changes
3. **Clean if needed**: Remove old workflows for clean slate
4. **Import desired set**: Use `import-enhanced` for development
5. **Verify operation**: Check status after operations

### **Production Deployment**
1. **Export current state**: Preserve existing configurations
2. **Clean instance**: Ensure clean deployment
3. **Import production workflows**: Use `import-all` for complete setup
4. **Verify functionality**: Test all imported workflows
5. **Create backup**: Backup successful deployment state

### **Maintenance Schedule**
- **Daily**: Status checks during active development
- **Weekly**: Full backups of working configurations
- **Before releases**: Complete export and backup cycles
- **After changes**: Verification of workflow integrity

## 🆘 Troubleshooting

### **Common Issues**

#### **"N8N container is not running"**
```bash
# Start N8N first
docker compose up -d n8n

# Then check status
./workflow_management_tools.sh status
```

#### **"Failed to import workflow"**
- Check workflow JSON syntax
- Ensure N8N has sufficient resources
- Try importing individual workflows

#### **"Permission denied"**
```bash
# Fix permissions
chmod +x workflow_management_tools.sh
chmod -R 755 n8n_data/ workflow_exports/ workflow_backups/
```

#### **"Database locked"**
- Wait for N8N operations to complete
- Restart N8N container if needed
- Check for concurrent operations

### **Recovery Procedures**

#### **After Failed Import**
```bash
# Clean and retry
./workflow_management_tools.sh clean
./workflow_management_tools.sh import-enhanced
```

#### **Restore from Backup**
```bash
# List available backups
ls -la workflow_backups/

# Copy workflows back to n8n_data/workflows/
cp workflow_backups/backup_20241101_143000/*.json n8n_data/workflows/

# Re-import
./workflow_management_tools.sh import-all
```

## 📈 Performance Notes

### **Import Performance**
- **Enhanced workflows**: ~10 workflows, ~30 seconds
- **All workflows**: ~33 workflows, ~60 seconds
- **Large imports**: May require N8N restart

### **Export Performance**
- **Database queries**: Fast, <5 seconds
- **File operations**: Depends on workflow count and size
- **API calls**: May be slower for large numbers of workflows

### **Optimization Tips**
- **Use enhanced import** for development (faster, focused)
- **Regular backups** prevent data loss
- **Monitor resource usage** during large operations
- **Schedule maintenance** during low-usage periods

---

## 🎉 Summary

The **N8N Workflow Management Tools** provide a comprehensive, safe, and efficient way to manage complex robot control workflows. With support for enhanced individual control mechanisms, automatic safety features, and robust error handling, this tool ensures reliable workflow deployment and maintenance.

**Ready for enterprise-level industrial automation!** 🚀🤖✨
