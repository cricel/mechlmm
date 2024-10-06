from setuptools import setup, find_packages

def read_requirements():
    with open('requirements.txt') as f:
        return f.read().splitlines()

setup(
    name="mechlmm_py",
    version="0.1",
    packages=find_packages(),
    install_requires=read_requirements(),
    # entry_points={
    #     'console_scripts': [
    #         'mechlmm_serve=mechlmm_py.mechlmm_server:main',
    #     ],
    # },
)