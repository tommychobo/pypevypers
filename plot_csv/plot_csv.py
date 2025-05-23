import pandas as pd
import matplotlib.pyplot as plt

def plot_csv_columns(file_path, x_column, y_column1, y_column2=None):
    # Load the CSV file
    try:
        data = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return
    except pd.errors.ParserError:
        print(f"Error parsing the file: {file_path}")
        return

    # Check if specified columns exist
    if x_column not in data.columns or y_column1 not in data.columns:
        print(f"Columns not found. Available columns: {list(data.columns)}")
        return

    # Plotting
    plt.figure(figsize=(8, 6))
    plt.plot(data[x_column], data[y_column1], marker='o')
    plt.plot(data[x_column], data[y_column2], marker='o') if ycolumn2 else None
    plt.title(f'{y_column1} vs {x_column}')
    plt.xlabel(x_column)
    plt.ylabel(y_column1)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Replace with your file path and column names
    plot_csv_columns("../csvs/test_1/051825Test1_22.csv", "time", "2p", "dp")