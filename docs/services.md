# Ros2 Service Server

> Activity: Using the number_counter node from the "ROS2 Publisher-Subscriber: Number Accumulator" assignment, creat a service that uses boolean values to restar the node counter.

---

**Visit the ROS2 Publisher-Subscriber: Number Accumulator assignment to check the codes in wich I worked upon in this assignment**

## Implementig and undestanding the service interface
The objective is to implement a service server that upon beeing called, will check the (bool data) from the request, then by (bool suceess) it will indicate a succesfull run of trigerred service, finally an informational message will be sent by (string message) to print any inidcation, error, or information that's considered important. All this using the arlready-installed interface:  
- ``` codigo
example_interfaces/srv/SetBool
```  
Expected output:  
``` code
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```