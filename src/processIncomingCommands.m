function processIncomingCommands(incoming_dir, fig)
% processIncomingCommands 處理 incoming/ 目錄中的指令文件
%   incoming_dir: 指令文件目錄
%   fig:          figure 句柄

    % 檢查忙碌狀態
    if fig.UserData.is_busy
        return;
    end
    
    files = dir(fullfile(incoming_dir, 'cmd_*.m'));
    if isempty(files)
        return;
    end
    
    [~, idx] = sort([files.datenum]);
    cmd_path = fullfile(incoming_dir, files(idx(1)).name);
    
    fprintf('[RX] %s\n', files(idx(1)).name);
    fig.UserData.is_busy = true;
    
    try
        run(cmd_path);
    catch ME
        fprintf('[ERR] %s\n', ME.message);
        failed_dir = fullfile(incoming_dir, 'failed');
        if ~exist(failed_dir, 'dir')
            mkdir(failed_dir);
        end
        movefile(cmd_path, fullfile(failed_dir, files(idx(1)).name));
        fig.UserData.is_busy = false;
        return;
    end
    
    % 執行成功後刪除
    if exist(cmd_path, 'file')
        delete(cmd_path);
    end
    
    fig.UserData.is_busy = false;
end
