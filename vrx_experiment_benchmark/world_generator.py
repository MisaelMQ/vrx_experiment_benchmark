#!/usr/bin/env python3
import os
import yaml
from jinja2 import Environment, FileSystemLoader
from ament_index_python.packages import get_package_share_directory


# Generate world SDF from environment preset yaml
def generate_world(env_name: str) -> str:
    pkg_share = get_package_share_directory('vrx_experiment_benchmark')

    env_file = os.path.join(pkg_share, 'config', 'env', f'{env_name}.yaml')
    template_dir = os.path.join(pkg_share, 'worlds')

    output_dir = '/tmp/vrx_experiment_benchmark/worlds'
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'sydney_regatta_benchmark.sdf')

    with open(env_file, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)

    jinja_env = Environment(
        loader=FileSystemLoader(template_dir),
        autoescape=False,
        trim_blocks=True,
        lstrip_blocks=True,
    )

    template = jinja_env.get_template('sydney_regatta_env_template.sdf.jinja')
    rendered = template.render(**params)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(rendered)

    return output_file


# CLI entrypoint
def main():
    import sys
    env_name = sys.argv[1] if len(sys.argv) > 1 else 'env_01_calm'
    world_path = generate_world(env_name)
    print(world_path)


if __name__ == '__main__':
    main()