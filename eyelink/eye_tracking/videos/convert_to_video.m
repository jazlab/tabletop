

filename = 't6_60820';
video = VideoWriter(strcat('eye_videos_v0/', filename),'MPEG-4');
video.FrameRate = 30;
buffer = load(strcat('matlab/', filename, '.mat')).frames;
open(video)
for i = 1:size(buffer,1)
    img = squeeze(buffer(i, :, :, :));
    writeVideo(video,img);
end
close(video);
