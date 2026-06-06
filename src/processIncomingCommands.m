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
    [~, cmd_name, ~] = fileparts(files(idx(1)).name);
    
    % 準備日誌目錄與路徑
    log_dir = fullfile(incoming_dir, '..', 'logs');
    if ~exist(log_dir, 'dir')
        mkdir(log_dir);
    end
    log_path = fullfile(log_dir, ['log_' datestr(now, 'yyyymmdd_HHMMSS_FFF') '.txt']);
    
    % 開始記錄命令窗口輸出（diary 會捕獲所有 fprintf / error / warning）
    diary off;
    diary(log_path);
    
    fprintf('[RX] %s\n', cmd_name);
    fig.UserData.is_busy = true;
    
    % 將關鍵變量注入函數工作空間，確保 run() 能找到
    % 注意: run() 在函數內部調用時，腳本運行在函數工作空間，不是 base workspace
    arm = fig.UserData.arm;
    current_q = fig.UserData.current_q;
    % fig 已經是函數參數，直接可用
    
    try
        run(cmd_path);
    catch ME
        % 詳細錯誤日誌：輸出到命令行（會被 diary 記錄）
        if ~isempty(ME.stack)
            err_msg = sprintf('[ERR] %s (line %d): %s', ME.stack(1).name, ME.stack(1).line, ME.message);
        else
            err_msg = sprintf('[ERR] %s', ME.message);
        end
        fprintf('%s\n', err_msg);
        
        % 保存到 incoming_history/
        history_dir = fullfile(incoming_dir, '..', 'incoming_history');
        if ~exist(history_dir, 'dir')
            mkdir(history_dir);
        end
        copyfile(cmd_path, fullfile(history_dir, [cmd_name '.m']));
        
        % 歸檔到 failed/
        failed_dir = fullfile(incoming_dir, 'failed');
        if ~exist(failed_dir, 'dir')
            mkdir(failed_dir);
        end
        movefile(cmd_path, fullfile(failed_dir, [cmd_name '.m']));
        
        diary off;
        printLog(cmd_name, log_path);
        fig.UserData.is_busy = false;
        return;
    end
    
    % 執行成功後保存到 history 再刪除
    history_dir = fullfile(incoming_dir, '..', 'incoming_history');
    if ~exist(history_dir, 'dir')
        mkdir(history_dir);
    end
    copyfile(cmd_path, fullfile(history_dir, [cmd_name '.m']));
    
    if exist(cmd_path, 'file')
        delete(cmd_path);
    end
    
    diary off;
    printLog(cmd_name, log_path);
    fig.UserData.is_busy = false;
end

function printLog(cmd_name, log_path)
% printLog 打印日誌文件內容到命令窗口
    fprintf('\n========== LOG: %s ==========\n', cmd_name);
    if exist(log_path, 'file')
        type(log_path);
    else
        fprintf('[WARN] Log file not found: %s\n', log_path);
    end
    fprintf('================================\n\n');
end
