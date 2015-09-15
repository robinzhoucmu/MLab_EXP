%SAVE2PDF Saves a figure as a properly cropped pdf
%
%   save2pdf(pdfFileName,handle,dpi)
%
%   - pdfFileName: Destination to write the pdf to.
%   - handle:  (optional) Handle of the figure to write to a pdf.  If
%              omitted, the current figure is used.  Note that handles
%              are typically the figure number.
%   - dpi: (optional) Integer value of dots per inch (DPI).  Sets
%          resolution of output pdf.  Note that 150 dpi is the Matlab
%          default and this function's default, but 600 dpi is typical for
%          production-quality.
%   - f_NO_CROP: (optional) If true, does not call pdfcrop at the end
%                 Defaults to false (does call pdfcrop).
%
%
%   Saves figure as a pdf with margins cropped to match the figure size.

%   (c) Gabe Hoffmann, gabe.hoffmann@gmail.com
%   Written 8/30/2007
%   Revised 9/22/2007
%   Revised 1/14/2007

function save2pdf(pdfFileName,handle,dpi, f_NO_CROP)

% Verify correct number of arguments
error(nargchk(0,4,nargin));
% If no handle is provided, use the current figure as default
if nargin<2
    handle = gcf;
end

if nargin<1
    [fileName,pathName] = uiputfile('*.pdf',['Save Fig. ' num2str(handle), ' to PDF file:']);
    if fileName == 0; return; end
    pdfFileName = [pathName,fileName];
end
[pathstr, name, ~] = fileparts(pdfFileName);
if (isempty(pathstr))
    pathstr = '.';
end
pdfFileName = [pathstr, '/', name, '.pdf'];


if (nargin<3 || isempty(dpi))
    dpi = 150;
end
if (nargin < 4)
    f_NO_CROP = true;
end

% Backup previous settings
prePaperType = get(handle,'PaperType');
prePaperUnits = get(handle,'PaperUnits');
preUnits = get(handle,'Units');
prePaperPosition = get(handle,'PaperPosition');
prePaperSize = get(handle,'PaperSize');

% Make changing paper type possible
set(handle,'PaperType','<custom>');

% Set units to all be the same
set(handle,'PaperUnits','inches');
set(handle,'Units','inches');

% Set the page size and position to match the figure's dimensions
paperPosition = get(handle,'PaperPosition');
position = get(handle,'Position');
set(handle,'PaperPosition',[0,0,position(3:4)]);
set(handle,'PaperSize',position(3:4));

% Save the pdf (this is the same method used by "saveas")
print(handle,'-dpdf',pdfFileName,sprintf('-r%d',dpi))
if (~f_NO_CROP)
    try
        system(['pdfcrop "' pdfFileName '" "' pdfFileName, '"']);
        delete('tmp-pdfcrop-*');
    catch err
    end
end

display(['Wrote PDF to ', pdfFileName]);

% Restore the previous settings
set(handle,'PaperType',prePaperType);
set(handle,'PaperUnits',prePaperUnits);
set(handle,'Units',preUnits);
set(handle,'PaperPosition',prePaperPosition);
set(handle,'PaperSize',prePaperSize);