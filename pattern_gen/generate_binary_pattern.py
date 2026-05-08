import numpy as np

def generate_ouster_os0_128(filename="OusterOS0_128.mat3x4f"):
    channels = 128
    horizontal_samples = 1024
    
    # OS0-128 vertical FOV: +45 to -45 degrees
    elevations = np.linspace(np.radians(45), np.radians(-45), channels)
    azimuths = np.linspace(0, 2 * np.pi, horizontal_samples, endpoint=False)
    
    num_rays = channels * horizontal_samples
    matrices = np.zeros((num_rays, 3, 4), dtype=np.float32)
    
    # Physical offsets matching your reference file
    offset_x = -0.012
    offset_z = 0.036

    idx = 0
    for az in azimuths:
        ca, sa = np.cos(az), np.sin(az)
        for el in elevations:
            ce, se = np.cos(el), np.sin(el)

            # Row 0: Transverse / Side
            matrices[idx, 0, 0] = sa
            matrices[idx, 0, 1] = se
            matrices[idx, 0, 2] = -ca * ce
            matrices[idx, 0, 3] = offset_x
            
            # Row 1: Forward (Using ca to match reference's ~1.0 value)
            matrices[idx, 1, 0] = ca
            matrices[idx, 1, 1] = -sa * se
            matrices[idx, 1, 2] = sa * ce
            matrices[idx, 1, 3] = 0.0
            
            # Row 2: Vertical / Up
            matrices[idx, 2, 0] = 0.0
            matrices[idx, 2, 1] = ce
            matrices[idx, 2, 2] = se
            matrices[idx, 2, 3] = offset_z
            
            idx += 1
            
    matrices.tofile(filename)
    print(f"Success. Generated {filename} with {num_rays} rays.")

if __name__ == "__main__":
    generate_ouster_os0_128()