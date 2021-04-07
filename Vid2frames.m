 clear variables; 
 vid=VideoReader('video.avi');
 numFrames = vid.NumFrames;
 n=numFrames;
 j=10;
 mkdir frames
 cd frames
 for i = 1:30:n
 frames = read(vid,i);
 imwrite(frames,['Image' num2str(j) '.jpg']);
 j=j+1;
 end
