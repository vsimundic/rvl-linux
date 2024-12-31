import pandas as pd

df = pd.read_csv('/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results_single_contact_handle_moveit.csv')


duplicates = df[df.duplicated(keep=False)]
duplicate_indices = duplicates.index.tolist()
print("Duplicate rows:")
print(duplicates)
print("\nIndices of duplicate rows:")
print(duplicate_indices)