import csv
import sys
import os

def is_nonzero(value):
    try:
        return float(value) != 0
    except ValueError:
        return False

def filter_csv(input_file):
    if not input_file.lower().endswith('.csv'):
        print("Error: Input file must be a CSV.")
        return

    output_file = os.path.splitext(input_file)[0] + '_filtered.csv'

    with open(input_file, 'r', newline='') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)

        for row in reader:
            if row and is_nonzero(row[-1]):
                writer.writerow(row)

    print(f"Filtered CSV saved as: {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python filter_nonzero_final_column.py <input_file.csv>")
        sys.exit(1)

    input_csv = sys.argv[1]
    filter_csv(input_csv)