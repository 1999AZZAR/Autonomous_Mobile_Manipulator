#!/bin/bash

# ROS2 Performance Testing Script
# Tests latency, throughput, and reliability of the ROS2 system

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Test results
declare -A test_results

echo "========================================"
echo "  ROS2 Performance Testing Suite"
echo "========================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ERROR: ROS2 not sourced${NC}"
    echo "Run: source /opt/ros/iron/setup.bash"
    exit 1
fi

echo -e "${GREEN}✅ ROS2 Distro: $ROS_DISTRO${NC}"
echo ""

# Test 1: Node Discovery Time
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 1: Node Discovery Time"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

START_TIME=$(date +%s.%N)
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
END_TIME=$(date +%s.%N)
DISCOVERY_TIME=$(echo "$END_TIME - $START_TIME" | bc)

echo "Nodes discovered: $NODE_COUNT"
echo "Discovery time: ${DISCOVERY_TIME}s"

if (( $(echo "$DISCOVERY_TIME < 2.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS - Fast discovery (<2s)${NC}"
    test_results["discovery"]="PASS"
else
    echo -e "${YELLOW}⚠️  WARN - Slow discovery (>2s)${NC}"
    test_results["discovery"]="WARN"
fi
echo ""

# Test 2: Topic List Performance
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 2: Topic List Performance"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

START_TIME=$(date +%s.%N)
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | wc -l)
END_TIME=$(date +%s.%N)
TOPIC_LIST_TIME=$(echo "$END_TIME - $START_TIME" | bc)

echo "Topics found: $TOPIC_COUNT"
echo "List time: ${TOPIC_LIST_TIME}s"

if (( $(echo "$TOPIC_LIST_TIME < 1.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS - Fast topic discovery (<1s)${NC}"
    test_results["topics"]="PASS"
else
    echo -e "${YELLOW}⚠️  WARN - Slow topic discovery (>1s)${NC}"
    test_results["topics"]="WARN"
fi
echo ""

# Test 3: Service Call Latency
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 3: Service Call Latency"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if robot status service exists
if ros2 service list | grep -q "get_robot_status"; then
    echo "Testing service: /get_robot_status"
    
    # Run 10 service calls and measure average
    TOTAL_TIME=0
    ITERATIONS=10
    
    for i in $(seq 1 $ITERATIONS); do
        START_TIME=$(date +%s.%N)
        timeout 2 ros2 service call /get_robot_status my_robot_automation/srv/GetRobotStatus "{}" > /dev/null 2>&1 || true
        END_TIME=$(date +%s.%N)
        CALL_TIME=$(echo "$END_TIME - $START_TIME" | bc)
        TOTAL_TIME=$(echo "$TOTAL_TIME + $CALL_TIME" | bc)
        echo -n "."
    done
    echo ""
    
    AVG_LATENCY=$(echo "scale=3; $TOTAL_TIME / $ITERATIONS" | bc)
    echo "Average latency: ${AVG_LATENCY}s (${ITERATIONS} calls)"
    
    if (( $(echo "$AVG_LATENCY < 0.05" | bc -l) )); then
        echo -e "${GREEN}✅ PASS - Low latency (<50ms)${NC}"
        test_results["service"]="PASS"
    elif (( $(echo "$AVG_LATENCY < 0.1" | bc -l) )); then
        echo -e "${YELLOW}⚠️  WARN - Medium latency (50-100ms)${NC}"
        test_results["service"]="WARN"
    else
        echo -e "${RED}❌ FAIL - High latency (>100ms)${NC}"
        test_results["service"]="FAIL"
    fi
else
    echo -e "${YELLOW}⚠️  SKIP - Service not available${NC}"
    test_results["service"]="SKIP"
fi
echo ""

# Test 4: Topic Publish Rate
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 4: Topic Publish Rate"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if ros2 topic list | grep -q "/cmd_vel"; then
    echo "Measuring /cmd_vel publish rate (5 seconds)..."
    
    # Measure for 5 seconds
    RATE_OUTPUT=$(timeout 5 ros2 topic hz /cmd_vel 2>/dev/null || echo "0.0")
    
    if echo "$RATE_OUTPUT" | grep -q "average rate"; then
        RATE=$(echo "$RATE_OUTPUT" | grep "average rate" | awk '{print $3}')
        echo "Publish rate: ${RATE} Hz"
        
        if (( $(echo "$RATE > 10.0" | bc -l) )); then
            echo -e "${GREEN}✅ PASS - Good publish rate (>10 Hz)${NC}"
            test_results["rate"]="PASS"
        elif (( $(echo "$RATE > 1.0" | bc -l) )); then
            echo -e "${YELLOW}⚠️  WARN - Low publish rate (1-10 Hz)${NC}"
            test_results["rate"]="WARN"
        else
            echo -e "${YELLOW}⚠️  INFO - Very low or no activity${NC}"
            test_results["rate"]="INFO"
        fi
    else
        echo -e "${YELLOW}⚠️  INFO - No messages published during test${NC}"
        test_results["rate"]="INFO"
    fi
else
    echo -e "${YELLOW}⚠️  SKIP - /cmd_vel topic not available${NC}"
    test_results["rate"]="SKIP"
fi
echo ""

# Test 5: Memory Usage
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 5: Memory Usage"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

USED_MEM=$(free -m | awk 'NR==2{print $3}')
TOTAL_MEM=$(free -m | awk 'NR==2{print $2}')
MEM_PERCENT=$(echo "scale=1; $USED_MEM * 100 / $TOTAL_MEM" | bc)

echo "Memory usage: ${USED_MEM}MB / ${TOTAL_MEM}MB (${MEM_PERCENT}%)"

if (( $(echo "$MEM_PERCENT < 70.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS - Healthy memory usage (<70%)${NC}"
    test_results["memory"]="PASS"
elif (( $(echo "$MEM_PERCENT < 85.0" | bc -l) )); then
    echo -e "${YELLOW}⚠️  WARN - High memory usage (70-85%)${NC}"
    test_results["memory"]="WARN"
else
    echo -e "${RED}❌ FAIL - Critical memory usage (>85%)${NC}"
    test_results["memory"]="FAIL"
fi
echo ""

# Test 6: CPU Load
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 6: CPU Load"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

CPU_LOAD=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
CPU_COUNT=$(nproc)
CPU_LOAD_PERCENT=$(echo "scale=1; $CPU_LOAD * 100 / $CPU_COUNT" | bc)

echo "CPU load: ${CPU_LOAD} (${CPU_LOAD_PERCENT}% of ${CPU_COUNT} cores)"

if (( $(echo "$CPU_LOAD_PERCENT < 60.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS - Healthy CPU load (<60%)${NC}"
    test_results["cpu"]="PASS"
elif (( $(echo "$CPU_LOAD_PERCENT < 85.0" | bc -l) )); then
    echo -e "${YELLOW}⚠️  WARN - High CPU load (60-85%)${NC}"
    test_results["cpu"]="WARN"
else
    echo -e "${RED}❌ FAIL - Critical CPU load (>85%)${NC}"
    test_results["cpu"]="FAIL"
fi
echo ""

# Test 7: DDS Configuration
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 7: DDS Configuration"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -n "$CYCLONEDDS_URI" ]; then
    echo -e "${GREEN}✅ CycloneDDS config: $CYCLONEDDS_URI${NC}"
    test_results["dds"]="PASS"
else
    echo -e "${YELLOW}⚠️  Using default DDS configuration${NC}"
    test_results["dds"]="WARN"
fi

if [ -n "$ROS_DOMAIN_ID" ]; then
    echo -e "${GREEN}✅ Domain ID: $ROS_DOMAIN_ID${NC}"
else
    echo -e "${YELLOW}⚠️  Using default domain ID (0)${NC}"
fi
echo ""

# Summary
echo "========================================"
echo "  Test Summary"
echo "========================================"
echo ""

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

for test in "${!test_results[@]}"; do
    result="${test_results[$test]}"
    case $result in
        "PASS")
            echo -e "${GREEN}✅ $test: PASS${NC}"
            ((PASS_COUNT++))
            ;;
        "WARN")
            echo -e "${YELLOW}⚠️  $test: WARN${NC}"
            ((WARN_COUNT++))
            ;;
        "FAIL")
            echo -e "${RED}❌ $test: FAIL${NC}"
            ((FAIL_COUNT++))
            ;;
        "SKIP"|"INFO")
            echo -e "${YELLOW}ℹ️  $test: ${result}${NC}"
            ;;
    esac
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Results: $PASS_COUNT passed, $WARN_COUNT warnings, $FAIL_COUNT failed"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Overall result
if [ $FAIL_COUNT -eq 0 ] && [ $WARN_COUNT -eq 0 ]; then
    echo -e "${GREEN}🎉 All tests passed! System is performing optimally.${NC}"
    exit 0
elif [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${YELLOW}⚠️  Some warnings detected. System is functional but could be optimized.${NC}"
    exit 0
else
    echo -e "${RED}❌ Some tests failed. Please review the results above.${NC}"
    exit 1
fi

