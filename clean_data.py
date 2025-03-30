import os
import shutil

def clean_evolution_data(root_dir):
    """
    Recursively remove evolution data files and folders from all subfolders
    """
    items_to_remove = {
        'files': ['evolution_data.csv'],
        'folders': ['saves', 'best_fitnesses']
    }
    
    files_removed = 0
    folders_removed = 0
    
    for root, dirs, files in os.walk(root_dir):
        # Remove files
        for file in files:
            if file in items_to_remove['files']:
                file_path = os.path.join(root, file)
                try:
                    os.remove(file_path)
                    files_removed += 1
                    print(f"Removed file: {file_path}")
                except Exception as e:
                    print(f"Error removing file {file_path}: {e}")
        
        # Remove folders
        for dir_name in dirs[:]:  # Copy list as we'll modify it
            if dir_name in items_to_remove['folders']:
                folder_path = os.path.join(root, dir_name)
                try:
                    shutil.rmtree(folder_path)
                    folders_removed += 1
                    print(f"Removed folder: {folder_path}")
                except Exception as e:
                    print(f"Error removing folder {folder_path}: {e}")
    
    print("\nCleanup Summary:")
    print(f"Total files removed: {files_removed}")
    print(f"Total folders removed: {folders_removed}")

if __name__ == "__main__":
    # Get the project root directory
    root_dir = os.path.dirname(os.path.abspath(__file__))
    
    print(f"This will remove all evolution_data.csv files and saves/best_fitnesses folders from:")
    print(root_dir)
    print("\nWARNING: This operation cannot be undone!")
    
    confirmation = input("\nDo you want to proceed? (yes/no): ")
    
    if confirmation.lower() == 'yes':
        clean_evolution_data(root_dir)
    else:
        print("Operation cancelled.")