from setuptools import setup, find_packages

setup(
    name="system-id ur5e",
    version="0.1.0",
    description="A small wrapper around UR5e RTDE control & receive interfaces",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Hamid Manouchehri",
    packages=find_packages(),            # auto‐finds the ur5ertde/ directory
    install_requires=[
        "numpy",
        "PyYAML",
        "matplotlib",
        "ur_rtde",
    ],
    python_requires=">=3.7,<3.11",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: MIT License"
    ],
)
