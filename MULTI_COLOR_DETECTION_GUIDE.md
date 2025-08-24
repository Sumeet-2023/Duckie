# Multi-Color Object Detection Guide

## Overview
The enhanced color detection system now supports 8 different colors with optimized HSV ranges and noise reduction for each color.

## Supported Colors

### 1. **Red** 🔴
- **Use cases**: Red balls, red objects, stop signs
- **HSV characteristics**: Dual range (0-10° and 160-180°)
- **Example targets**: `red_ball`, `red_object`, `red_cone`

### 2. **Blue** 🔵
- **Use cases**: Blue balls, blue markers, sky objects
- **HSV characteristics**: Single range (100-130°)
- **Example targets**: `blue_ball`, `blue_object`, `blue_marker`

### 3. **Green** 🟢
- **Use cases**: Green balls, vegetation, go signals
- **HSV characteristics**: Single range (40-80°)
- **Example targets**: `green_ball`, `green_object`, `green_cone`

### 4. **Yellow** 🟡
- **Use cases**: Yellow balls, caution markers, bananas
- **HSV characteristics**: Single range (20-30°)
- **Example targets**: `yellow_ball`, `yellow_object`, `yellow_cone`

### 5. **Orange** 🟠
- **Use cases**: Orange balls, traffic cones, construction markers
- **HSV characteristics**: Single range (8-20°)
- **Example targets**: `orange_ball`, `orange_cone`, `orange_object`

### 6. **Purple** 🟣
- **Use cases**: Purple balls, decorative objects
- **HSV characteristics**: Single range (130-160°)
- **Example targets**: `purple_ball`, `purple_object`, `purple_marker`

### 7. **Cyan** 🩵
- **Use cases**: Cyan balls, aqua objects, pool markers
- **HSV characteristics**: Single range (80-100°)
- **Example targets**: `cyan_ball`, `cyan_object`, `cyan_marker`

### 8. **Pink** 🩷
- **Use cases**: Pink balls, decorative objects
- **HSV characteristics**: Dual range (160-180° and 0-10°)
- **Example targets**: `pink_ball`, `pink_object`, `pink_marker`

## Usage Examples

### 1. **Launch with Different Colors**
```bash
# Default red detection
roslaunch autonomous_following.launch

# Blue ball detection
roslaunch autonomous_following.launch target_color:=blue_ball

# Green object detection  
roslaunch autonomous_following.launch target_color:=green_object

# Yellow cone detection
roslaunch autonomous_following.launch target_color:=yellow_cone

# Orange ball detection
roslaunch autonomous_following.launch target_color:=orange_ball

# Purple marker detection
roslaunch autonomous_following.launch target_color:=purple_marker

# Cyan ball detection
roslaunch autonomous_following.launch target_color:=cyan_ball

# Pink object detection
roslaunch autonomous_following.launch target_color:=pink_object
```

### 2. **Debug Different Colors**
```bash
# Debug red detection (default)
python3 debug_detection.py

# Debug blue detection
python3 debug_detection.py _target_color:=blue_ball

# Debug green detection  
python3 debug_detection.py _target_color:=green_object

# Debug yellow detection
python3 debug_detection.py _target_color:=yellow_cone

# Debug orange detection
python3 debug_detection.py _target_color:=orange_ball

# Debug purple detection
python3 debug_detection.py _target_color:=purple_marker

# Debug cyan detection
python3 debug_detection.py _target_color:=cyan_ball

# Debug pink detection
python3 debug_detection.py _target_color:=pink_object
```

### 3. **ROS Parameter Setting**
```bash
# Set target color via ROS parameter
rosparam set /color_object_detection_node/target_color blue_ball

# Check current target color
rosparam get /color_object_detection_node/target_color

# List all supported colors from config
rosparam get /detection/supported_colors
```

## Color Range Details

### Strictness Levels
Each color has three strictness levels that are automatically selected based on detected pixel count:

1. **Strict** (500+ pixels required)
   - High saturation (150+) and value (150+)
   - Best for bright, clear objects in good lighting
   - Minimal noise but may miss dim objects

2. **Relaxed** (200+ pixels required)
   - Medium saturation (80-100+) and value (100+)
   - Balanced detection for normal conditions
   - Good compromise between detection and noise

3. **Very Relaxed** (50+ pixels required)
   - Low saturation (30-50+) and value (50+)
   - Catches dim/distant objects but more noise
   - Used only when other levels fail

### Automatic Range Selection Logic
```python
if strict_pixels > 500:      # Prefer strict for clean detection
    use_strict_range()
elif relaxed_pixels > 200:   # Fall back to relaxed
    use_relaxed_range()
else:                        # Last resort
    use_very_relaxed_range()
```

## Performance Optimization Tips

### 1. **Lighting Conditions**
- **Bright outdoor**: Use with any color, system will prefer strict ranges
- **Indoor/dim**: System automatically falls back to relaxed ranges
- **Mixed lighting**: Relaxed ranges work best
- **Very dark**: May need to increase very_relaxed thresholds

### 2. **Object Properties**
- **Bright/saturated objects**: Detected with strict ranges (best performance)
- **Pale/washed out objects**: Detected with relaxed ranges
- **Very dim objects**: May only be detected with very_relaxed ranges

### 3. **Camera Settings**
- **Auto white balance**: Generally works well
- **Manual white balance**: May need HSV range adjustments
- **High contrast**: Better detection performance
- **Low contrast**: May need relaxed ranges

## Troubleshooting

### Common Issues:

1. **"Unsupported target color" error**
   ```bash
   # Check if color name is correct
   echo "Supported: red, blue, green, yellow, orange, purple, cyan, pink"
   # Make sure target format is: color_object (e.g., blue_ball)
   ```

2. **No detections found**
   ```bash
   # Run debug to see pixel counts
   python3 debug_detection.py _target_color:=your_color_object
   # Check if any range shows >50 pixels
   ```

3. **Too many false detections**
   ```bash
   # Try increasing strictness thresholds in config
   # Or improve lighting conditions
   # Check morphological kernel size
   ```

4. **Detection too sensitive/not sensitive enough**
   ```bash
   # Adjust HSV ranges in autonomous_config.yaml
   # Modify min_pixels thresholds for each strictness level
   # Tune morphological operations
   ```

## Monitoring Performance

### 1. **Detection Logs**
```bash
# Watch detection performance
rostopic echo /pinkduckie/object_detection/detection_data

# Monitor node logs
tail -f /tmp/log/latest/color_object_detection_node*.log
```

### 2. **Debug Output Analysis**
Look for these key metrics in debug output:
- **Pixel counts per range**: Should show progression strict < relaxed < very_relaxed
- **Valid contour ratio**: Should be >50% for good performance
- **Selected range**: Should prefer strict when possible

### 3. **Performance Indicators**
- **Good performance**: 5-20 contours total, 60%+ valid ratio, using strict/relaxed ranges
- **Fair performance**: 20-50 contours total, 30-60% valid ratio, using relaxed ranges  
- **Poor performance**: 50+ contours total, <30% valid ratio, using very_relaxed ranges

## Configuration Tuning

Edit `/home/sumeettt/errorProject/Duckie/config/autonomous_config.yaml` to adjust:

- **HSV ranges** for each color and strictness level
- **Minimum pixel thresholds** for range selection
- **Morphological kernel sizes** for noise reduction
- **Contour filtering criteria** (aspect ratio, solidity, etc.)

## Next Steps

1. **Test each color** with real objects in your environment
2. **Fine-tune HSV ranges** based on your lighting conditions
3. **Adjust strictness thresholds** for optimal performance
4. **Add new colors** by extending the color_ranges dictionary