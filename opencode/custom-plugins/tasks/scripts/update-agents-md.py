#!/usr/bin/env python3
"""
Update AGENTS.md with tasks tools guide.
This script inserts or updates the guarded TASKS_TOOLS_GUIDE block in AGENTS.md.
"""

import sys
import re
import shutil
from datetime import datetime

def update_agents_md(agents_md_path, template_path):
    """Update AGENTS.md with tasks tools guide from template."""
    
    # Read template content
    with open(template_path, 'r') as f:
        tasks_guide = f.read()
    
    # Backup existing AGENTS.md
    backup_path = f"{agents_md_path}.backup.{datetime.now().strftime('%Y%m%d%H%M%S')}"
    shutil.copy(agents_md_path, backup_path)
    print(f"✓ Backup created: {backup_path}")
    
    # Read current AGENTS.md
    with open(agents_md_path, 'r') as f:
        content = f.read()
    
    # Check if guarding block already exists
    if "<!-- TASKS_TOOLS_GUIDE_START -->" in content:
        # Replace existing block
        pattern = r'<!-- TASKS_TOOLS_GUIDE_START -->.*?<!-- TASKS_TOOLS_GUIDE_END -->'
        new_content = re.sub(pattern, tasks_guide, content, flags=re.DOTALL)
        
        with open(agents_md_path, 'w') as f:
            f.write(new_content)
        print("✓ Existing tasks guide block updated")
    else:
        # Add new block after "Global Tools & Skills" section
        target = "## 3. Global Tools & Skills (글로벌 도구 및 스킬)"
        
        if target in content:
            lines = content.split('\n')
            insert_idx = None
            
            for i, line in enumerate(lines):
                if target in line:
                    # Find next section starting with ##
                    for j in range(i + 1, len(lines)):
                        if lines[j].startswith('## ') and j > i:
                            insert_idx = j
                            break
                    break
            
            if insert_idx:
                lines.insert(insert_idx, '')
                lines.insert(insert_idx + 1, tasks_guide)
                new_content = '\n'.join(lines)
                
                with open(agents_md_path, 'w') as f:
                    f.write(new_content)
                print("✓ Tasks guide added after Global Tools & Skills section")
            else:
                # Append to end
                with open(agents_md_path, 'a') as f:
                    f.write('\n\n')
                    f.write(tasks_guide)
                print("✓ Tasks guide appended to end of file")
        else:
            # Append to end
            with open(agents_md_path, 'a') as f:
                f.write('\n\n')
                f.write(tasks_guide)
            print("✓ Tasks guide appended to end of file")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: update-agents-md.py <AGENTS_MD_PATH> <TEMPLATE_PATH>")
        sys.exit(1)
    
    agents_md_path = sys.argv[1]
    template_path = sys.argv[2]
    
    update_agents_md(agents_md_path, template_path)
