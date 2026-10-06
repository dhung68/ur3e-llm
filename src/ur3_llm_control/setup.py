from setuptools import setup

setup(
    name="ur3_llm_control", version="0.1.0", packages=["ur3_llm_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ur3_llm_control"]),
        ("share/ur3_llm_control", ["package.xml"]),
    ],
    package_data={"ur3_llm_control": ["planner_prompt.txt", "bai3_prompt.txt"]},
    install_requires=["setuptools"], zip_safe=True,
    maintainer="HRI student", maintainer_email="student@example.com",
    description="Physical Gazebo pick/place skills with measured object state.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "bai3_task = ur3_llm_control.bai3_task:main",
        "llm_task = ur3_llm_control.llm_node:main",
        "run_skills = ur3_llm_control.cli:main",
    ]},
)
