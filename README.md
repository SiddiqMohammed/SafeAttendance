# SafeAttendance

Facial recognition using openCV python and contactless temperature sensing with mlx90614 to provide a safe workplace attendance system.

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install the following .

```bash
pip install smbus2 
pip install mlx90614 
pip install face_recognition
pip install numpy
pip install opencv-python
pip install opencv-contrib-python
```

## Usage

Make a folder named "ImagesAttendance" where you will store your source images of the people you want to recognize the face of.
Create a csv file named "Attendance.csv" in the same folder as the main.py file.

When a person is detected for 5 consecutive frames then the temperature is checked using the mlx9610 contactless temp sensor that is connected an arduino and the data is sent to the python code using serial communication.



## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
