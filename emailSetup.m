% Your Gmail address and app password
myAddress  = 'youremail@gmail.com';
myPassword = '16-character-app-password';  % Gmail app-password

% Email preferences
setpref('Internet','E_mail',        myAddress);
setpref('Internet','SMTP_Server',   'smtp.gmail.com');
setpref('Internet','SMTP_Username', myAddress);
setpref('Internet','SMTP_Password', myPassword);

% Java mail properties
props = java.lang.System.getProperties;
props.setProperty('mail.smtp.auth',              'true');
props.setProperty('mail.smtp.starttls.enable',   'true');
props.setProperty('mail.smtp.port',              '587');  % STARTTLS