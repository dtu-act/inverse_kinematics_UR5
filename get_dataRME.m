function [data,flagRep] = get_dataRME(playRec,audioToPlay,maxRep)
%[data,flagRep] = get_dataRME(playRec,audioToPlay,maxRep) Playback and
% record simultaneously a signal given in audioToPlay

[frameSize,nFrames] = size(audioToPlay);
audioRecorded = nan(frameSize,nFrames);
rep = 1;
while rep <= maxRep
    flagRep = 0;
    for frame = 1:nFrames
        [audioRecorded(:,frame),nUnderruns,nOverruns] = playRec(audioToPlay(:,frame));

        if nUnderruns > 0
            fprintf('Audio player queue was underrun by %d samples.\n',nUnderruns);
            flagRep = 1;
        end
        if nOverruns > 0
            fprintf('Audio recorder queue was overrun by %d samples.\n',nOverruns);
            flagRep = 1;
        end
    end
    release(playRec)

    if flagRep == 1
        if rep < maxRep
            rep = rep + 1;
        else
            break;
        end
    else
        break;
    end
end
data = audioRecorded(:);

end