import numpy as np

def compare_presets(file_custom, file_existing, x=1):
    def read_file(fname):
        return np.fromfile(fname, dtype=np.float32, count=x*12).reshape(-1, 3, 4)

    custom = read_file(file_custom)
    existing = read_file(file_existing)

    for i in range(min(len(custom), len(existing))):
        print(f"=== Entry {i} Comparison ===")
        print(f"{'CUSTOM (128)':<20} | {'EXISTING (64)':<20}")
        print("-" * 50)
        for row in range(3):
            c_row = str(custom[i, row])
            e_row = str(existing[i, row])
            print(f"{c_row:<20} | {e_row:<20}")
        print("\n")

if __name__ == "__main__":
    # Ensure filenames match your local files
    compare_presets("../lidar_patterns/OusterOS0_128.mat3x4f", "../lidar_patterns/OusterOS1_64.mat3x4f")