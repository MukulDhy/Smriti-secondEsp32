import os
import time

def extract_metadata(file_path):
    # File metadata
    size = os.path.getsize(file_path)
    created = time.ctime(os.path.getctime(file_path))
    modified = time.ctime(os.path.getmtime(file_path))
    file_type = os.path.splitext(file_path)[-1]

    return {
        "File Name": os.path.basename(file_path),
        "Size": f"{size} bytes",
        "Created": created,
        "Last Modified": modified,
        "File Type": file_type,
        "Full Path": file_path
    }

def write_folder_data(folder_path, output_file):
    with open(output_file, 'w', encoding='utf-8') as output:
        for root, dirs, files in os.walk(folder_path):
            for file in files:
                file_path = os.path.join(root, file)
                
                # Extract and write metadata
                metadata = extract_metadata(file_path)
                output.write("=== FILE METADATA ===\n")
                for key, value in metadata.items():
                    output.write(f"{key}: {value}\n")
                
                # Read and write file content
                output.write("\n--- FILE CONTENT START ---\n")
                try:
                    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                        output.write(content)
                except Exception as e:
                    output.write(f"[Error reading file: {e}]\n")
                output.write("\n--- FILE CONTENT END ---\n\n")
                
                output.write("=" * 40 + "\n\n")

    print(f"All data written to {output_file}")

# Example usage
if __name__ == "__main__":
    folder = r"C:\Users\mukul\OneDrive\Documents\PlatformIO\Projects\esp2ndDevice\src"  # Replace with your folder path
    output_file = "output.txt"
    write_folder_data(folder, output_file)
