import csv

def convert_csv_to_cpp_array(csv_file):
    waypoints = []
    
    with open(csv_file, 'r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            # Convert string values to appropriate types and format
            x = float(row['X'])
            y = float(row['Y'])
            z = float(row['Z'])
            rotation = float(row['Rotation'])
            theta1 = float(row['Theta1'])
            theta2 = float(row['Theta2'])
            theta3 = float(row['Theta3'])
            stop = 1.0 if int(row['StopAtPoint']) == 1 else 0.0
            duration = float(row['Duration'])
            linear = 1.0 if int(row['LinearPath']) == 1 else 0.0
            
            # Format each waypoint
            waypoint = f"    {{{x:.6f}, {y:.6f}, {z:.6f}, {rotation:.6f}, {theta1:.6f}, {theta2:.6f}, {theta3:.6f}, {stop:.1f}, {duration:.6f}, {linear:.1f}}}"
            waypoints.append(waypoint)
    
    # Join all waypoints with line breaks and add the final formatting
    waypoints_str = ",\n".join(waypoints)
    num_waypoints = len(waypoints)
    cpp_array = f"""const float waypoints[][10] = {{
{waypoints_str}
}};"""
    
    return cpp_array

# Example usage
if __name__ == "__main__":
    csv_file = "complex_XYZ_waypoint_test.csv"  # Replace with your CSV file path
    cpp_array = convert_csv_to_cpp_array(csv_file)
    print(cpp_array)