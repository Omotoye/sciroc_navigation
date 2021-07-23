<a href="https://unige.it/en/">
<img src="images/unige.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>
<a href="https://sciroc.org/2021-challenge-description/">
<img src="images/sciroc-logo.png" width="20%" height="20%" title="SciRoc Challenge 2021" alt="SciRoc Challenge 2021" >
</a>

# SciRoc Challenge 2021 (Robotics Engineering / JEMARO, Unige)

**Author's Name: Omotoye Shamsudeen Adekoya**

**Student ID: 5066348**

---

This package contains is a submodule to a Behaviour Logic Package, it acts as a component that would be called to navigate a TIAGO robot (_provided for the SciRoc Challenge_) to a predetermined **Point of Interest** in the map.  

# ROS Package Description  
This package simply implements a ros service that accepts a request of the format below, which contains a string message representing the ID of the point of interest (poi).
```srv
string goal # This is the id of the Point of Interest 
---
string result
```

 It checks if the given poi ID is a valid one;
```python
rospy.has_param('/mmap/poi/submap_0/{goal}'.format(goal=req.goal))
```
if the _yes_, it then sends the point of interest to an already provided action server called _**go_to_poi**_; this action server help to navigate to the corresponding point. 


