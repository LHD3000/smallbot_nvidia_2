from setuptools import find_packages, setup

package_name = "toolbox"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="czq",
    maintainer_email="1448973286@qq.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plot_file = toolbox.plot.plot_file:main",
            "plot_runtime = toolbox.plot.plot_runtime:main",
            "plot_test=toolbox.plot.topic_pub_test:main",
            "odom2tf=toolbox.odom2tf:main",
        ],
    },
)
