from setuptools import setup

package_name = 'usb_cam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['usb_cam.scripts.save_images_from_bag'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'save_images_from_bag = usb_cam.scripts.save_images_from_bag:main',
        ],
    },
)
