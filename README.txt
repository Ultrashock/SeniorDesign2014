Welcome to UNCA/NCSU Mechatronics Engineering Senior Design Students!!
-------------------------------------------------------------------------
Here is a guide brought to you by the 2014 Senior Design Team, to bring you up to speed on where we left off in the project.

NOTE: all code should be fully commented, which hopefully makes it understandable.


MASTER CONTROLLER FOLDER:
This is where each iteration of the code is stored, the versioning of the code was to be worked out a little better, hence the x's where numbers would normally go.

ENCODER TESTING:
The new encoders were slightly buggy, a testbench as well as some arduino code is in there.

FINAL REPORT:
This contains a LaTeX implementation of our final report, we used Texmaker and Miktex to compile our report.  Although these could have been organized better, do not change the formatting as the pictures in the report reference the path of where they are now.

SENIOR DESIGN PRESENTATION:
This uses a specific implementation of LaTeX called "beamer" it is specifically used to make presentation slides, and looks amazing when used properly.  The files were adapted from the "Radbound University Style"  It still contains the initial filenames from the template, hence the oddness of the names. "example.pdf" is the outputted file.  The special header that contains the NCSU and the UNC-Asheville logos was designed by Brandon Zschokke for this presentation, recycle and reuse it for future presentations if needed. Open this file solely in Adobe or Foxit reader, google viewer will cause the watermarks to have zero transparency.

SIMULINK FILE:
The simulink file was taken from the Mathworks Website and slightly adapted to give an idea of the differential control scheme for the project.  The file can be found in simulink under the help menu.  I believe it is called "4wd electric differential."  Matlab 2014 now provides support for the arduino due, so it is extremely possible for this to be adapted to work with the controller as opposed to being used solely for design guidance.

DATA ACQUISITION:
These are CSV files that contain information acquired through putting the car through various tests.  Information about these tests can be seen in the final report.

DOCUMENTS+REFERENCE MATERIAL:
This is a collection of data that was useful for us in designing the system, it may help refine the design later on in the future as well.


HOW TO MAKE THIS PROJECT YOUR OWN:
Fork this repository into your own project.
https://help.github.com/articles/fork-a-repo

I highly recommend using github for versioning as well as fixing issues with future coding.  Learn from our mistakes and make sure everyone is on board with how to use it before beginning, it has a learning curve and takes some getting used to, but is a very powerful tool when used correctly!  There is a reason tons of large projects are hosted on github.

Any questions??
Email:  bzschokk (at) alumni.unca
	blzschok (at) alumni.ncsu


-------------------------------------------------------------------------


                ##################                
               #;.,.,.,...,.,.,.,,#               
              #':;;;;;;;;;;;;;;;;;:#              
             #+,;;;;;;;;;;;;;;;;;;;;#             
            #+.;;;;;;;;;;;;;;;;;;;;:'#            
            #`;;;;;:;:;;;;;;:::;;;;;,#.           
            #,;;;;;,;; ,,,,,;;`;;;;;;#.           
            #,;;;;;,;;;.;###;;`;;;;;;#.           
            #,;;;;;,;;;;;`##;;`;;;;;;#.           
            #,;;;;;,;,`;;; #;;`;;;;;;#.           
            #,;;;;;,;,# ;;;`;;`......#.           
            #,;;;;;,;,##`;;;;;`#######.           
            #,;;;;;,;,###;.;;;`###.               
            #,;;;;;,;,::::: ;;`:: #`              
            #.;;;;;;;;;;;;;;;;;;;;`#`             
            #+.;;;;;;;;;;;;;;;;;;;;`#`            
             #+,;;;;;;;;;;;;;;;;;;;;`#            
              #+,;;;.        ,;;;;;;;#.           
               #'.. ;;;;;;;;;; ;;;;;;#.           
                ###,;;;;;;;;;;`;;;;;;#.           
            #######,;,++++++;;`;;;;;;#.           
            # `````,;,######''`;;;;;;#.           
            #,;;;;;,;,######.. ;;;;;;#.           
            #,;;;;;,;,::::::;;`;;;;;;#.           
            #,;;;;;,;;;;;;;;;;`;;;;;;#.           
            #,;;;;; ;;;;;;;;;; ;;;;;;#.           
            #,;;;;;;:        :;;;;;;;#.           
            #:;;;;;;;;;;;;;;;;;;;;;;.#            
             #:;;;;;;;;;;;;;;;;;;;;`#             
              #,;;;;;;;;;;;;;;;;;;`#              
               #,;;;;;;;;;;;;;;;;`#
                ##################`;              
                 ````````````````            
                                        
                                     `                                              
                                    ``                                              
                                 ``````                                             
                                 ``````                                             
                                ````````                                            
                                ````````                                            
                                `````````                                           
                               ``````````                                           
                               ``` ``````                                           
                              ````  ``````                                          
                              ```   ``````                                          
                              ```    ``````                                         
                             ````    ``````                                         
                             ```     ```````                                        
                             ```      ``````                                        
                            ```       ``````                                        
                            ```        ``````                                       
                           ````        ``````                                       
                           ```          ``````                                      
                           ```          ``````                                      
                          ```           ```````                                     
                          ```            ``````                                     
                         ````            ```````                                    
                         ```              ``````                                    
                         ```              ``````                                    
                        ````               ``````                                   
                        ```                ``````         `````````````             
                       ````                 ``````    ````````       `````          
                       ```                  `````` ``````                 ``        
                       ```                  ``````````                      ``      
                      ````                   ```````                          ``    
                      ```                   `````                              ``   
                     ````                  ````          ```````````             `  
                     ```                 `````        ``````````````````          ` 
                     ```                ````       ````````            ```        ` 
                    ````              `````      `````                              
                    ```              ````      ```````                              
                   ````             ````     `````````                              
                   ```            ````     ```` ``````                              
                   ```           ````     ```    ``````                             
                  ````         ````     ```      ``````                             
                  ```         ````    ```         ``````                            
                  ```       ````     ``           ``````                            
                 ```       ````    ```            ```````                           
                 ```     ````     ``               ``````                           
                ````   ````      `                 ``````                           
                ````  ````      `                  ```````                          
               `````````                            ```````                         
  `            ``````                              ````````                         
    ``       ``````                               ```````````
     `````````                                 ````````````````      

Thats all folks!     

=======
>>>>>>> Final Archive
