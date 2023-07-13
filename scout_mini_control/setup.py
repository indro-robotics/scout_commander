from setuptools import setup
import os

package_name = 'scout_mini_control'
include_package = 'scout_mini_control/include'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, include_package],
    data_files=package_files(data_files, ['launch/', 'models/', 'config/', 'maps/', 'params/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liamd',
    maintainer_email='liamd@indrorobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scout_mini_control = scout_mini_control.scout_mini_control:main',
            'waypoint_follower = scout_mini_control.waypoint_follower:main',
            'return_to_home = scout_mini_control.include.return_to_home:main',
        ],
    },
)
