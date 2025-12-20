import re
import os

def add_front_matter_to_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Regex to find existing front matter block (if any)
    # Use re.DOTALL to make '.' match newlines
    front_matter_regex = re.compile(r"^(---.*?---)", re.DOTALL)
    
    existing_front_matter_block = None
    # Check if content starts with '---' to indicate potential front matter
    if content.startswith('---'):
        match = front_matter_regex.match(content)
        if match:
            existing_front_matter_block = match.group(1)
            # Remove existing front matter from content for clean re-addition
            content = content[len(existing_front_matter_block):] 

    # Extract ID from filename
    file_name_without_ext = os.path.splitext(os.path.basename(file_path))[0]
    doc_id = file_name_without_ext.replace(' ', '-')

    # Extract Title from the first H1 heading or generate from ID
    # Search in the content *after* potential front matter removal
    first_h1_match = re.search(r'^#\s*(.+)', content, re.MULTILINE)
    if first_h1_match:
        title = first_h1_match.group(1).strip()
    else:
        title = ' '.join([word.capitalize() for word in doc_id.split('-')])

    # Enclose title in double quotes for YAML compatibility
    quoted_title = f'"{title}"'

    # Construct new front matter
    new_front_matter = f"---\nid: {doc_id}\ntitle: {quoted_title}\n---\n\n"
    new_content = new_front_matter + content

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Processed front matter for {file_path}")

# List of files obtained from glob command
markdown_files = [
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\01-introduction.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\02-nodes-and-topics.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\03-first-ros-program.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\04-services-and-actions.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\05-building-a-service.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\06-intro-to-urdf.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\07-bridging-ai-to-ros.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-1-ros2\08-review-and-next-steps.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\01-introduction-to-digital-twins.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\02-gazebo-architecture-and-workflow.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\03-physics-simulation.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\04-sensor-simulation.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\05-gazebo-vs-unity.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-2-digital-twin\06-review-and-next-steps.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\00-review-and-next-steps.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\01-isaac-overview.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\02-synthetic-data.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\03-isaac-ros-perception.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\04-vslam-localization.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\05-navigation-planning.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-3-isaac\06-sim-to-real.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\00-review-and-next-steps.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\01-vla-philosophy.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\02-voice-to-text.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\03-language-to-plan.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\04-orchestrator.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\05-vision-grounding.md',
    r'C:\Users\Wajiz.pk\OneDrive\Desktop\GIAIC\Physical-AI-Textbook\website\docs\module-4-vla\06-capstone-project.md',
]

for file_path in markdown_files:
    add_front_matter_to_file(file_path)

print("Finished processing front matter for all markdown files.")