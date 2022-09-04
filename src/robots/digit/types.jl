struct Digit
    id::Int64
    p
    joint_names
    joint_ids

    function Digit(id, p)
        joint_names = SA["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
            "left-knee", "left-shin", "left-tarsus", "left-toe-pitch", 
            "left-toe-roll", "left-shoulder-roll", "left-shoulder-pitch", 
            "left-shoulder-yaw", "left-elbow", "right-hip-roll", "right-hip-yaw", 
            "right-hip-pitch", "right-knee", "right-shin", "right-tarsus", 
            "right-toe-pitch", "right-toe-roll", "right-shoulder-roll", 
            "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        joint_ids = [14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 22, 23, 24, 25, 26, 27, 28, 29, 6, 7, 8, 9]
        new(id, p, joint_names, joint_ids)
    end
end