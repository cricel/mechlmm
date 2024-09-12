
elapsed_time = 10
# current_reference_videos = []
current_reference_videos = [['video_file_name', '30', '30']]
# current_reference_videos = [['video_file_name', '30', '30'], ['video_file_name', '90', '90']]
print("-=-=-reference_videos-=-=-=-")
print(current_reference_videos)

if(current_reference_videos == []):
    current_reference_videos.append(["video_file_name", str(elapsed_time), str(elapsed_time)])
else:
    if(elapsed_time - int(current_reference_videos[-1][1]) < 30):
        current_reference_videos[-1][2] = str(elapsed_time)
    else:
        current_reference_videos.append(["video_file_name", str(elapsed_time), str(elapsed_time)])

print(current_reference_videos)