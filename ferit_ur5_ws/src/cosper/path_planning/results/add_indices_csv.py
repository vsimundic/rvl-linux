import csv

# Input and output file names
input_file = "/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handleless_bVNPanelTrue.csv"  # Change this to your actual input file name
output_file = "/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handleless.csv"  # Change this to your desired output file name

# Read the CSV file and add an index
with open(input_file, mode='r', newline='', encoding='utf-8') as infile:
    reader = csv.reader(infile)
    rows = list(reader)
    
    # Ensure the file is not empty and has a header
    if not rows:
        print("Error: The CSV file is empty.")
        exit()
    
    header = ["idx"] + rows[0]  # Add index column header
    data = [[i] + row for i, row in enumerate(rows[1:])]  # Add index to each row

# Write the modified data back to a new CSV file
with open(output_file, mode='w', newline='', encoding='utf-8') as outfile:
    writer = csv.writer(outfile)
    writer.writerow(header)
    writer.writerows(data)

print(f"File successfully saved as {output_file}")
