## Code Structure

    ├── Cleaning robot based on object detection     # Root directory of the project.
        ├── dataset                         # Directory containing labeled dataset.
        |   ├── img1.jpg       
        |   ├── img1.xml       
        |   ├── img2.jpg       
        |   ├── img2.xml       
            ...       
        |   └── img550.xml        
        ├── src                     # Directory containing the majority of the implemented code
        |   ├── custom_model_lite
            |   ├── labelmap.txt
            |   ├── detect.tflite 
            |   └── ..               
        |   ├── main.py     
        |   ├── TFLite_detection_webcam.py
        |   ├── LEDcomtrol.py                    
        |   └── predefined_class.txt                
        ├── src_motor                  # Directory containing the code to process motor
        |   ├── measureDistance.py
        |   ├── move.py
        |   ├── HR8825.py 
        |   └──  main.py
        |               
        └──README.md  #Record the code structure       
