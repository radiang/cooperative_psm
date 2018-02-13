function [hB, hF] = delete_beam_segments(blks)

for i = 2:length(blks)
    try                                                 %#ok
        if strcmp(get_param(blks(i),'type'),'line')
            delete_line(blks(i))
        end
        if strcmp(get_param(blks(i),'type'),'block')
            % Store handle to Connection ports
            if strcmp(get_param(blks(i),'name'),'B')
                hB = blks(i);
            elseif strcmp(get_param(blks(i),'name'),'F')
                hF = blks(i);
            else
                delete_block(blks(i))
            end
        end
    end
end
   


