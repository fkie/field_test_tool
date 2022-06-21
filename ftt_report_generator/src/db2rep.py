#!/usr/bin/env python
"""db2rep.py: Automatic report generator out of the data stored in the FTT database."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import os
import sys
from lxml import etree  # see http://lxml.de/tutorial.html
from jinja2 import Environment, FileSystemLoader  # see http://jinja.pocoo.org
import base64
import time
import datetime
import parse
from subprocess import call
from dbAdapter import FttAdapter
from mapGenerator import MapGenerator
from plotsGenerator import PlotsGenerator

builddir = "../build"
imagedir = "images"
maptiledir = "map_tiles"
logosdir = "../src/logos"
DEFAULT_HEADER_LATEX_TEMPLATE = "rep_header_template.j2"
DEFAULT_SHIFT_HEADER_TEMPLATE = "rep_shift_header_template.j2"
DEFAULT_SHIFT_STATISTICS_HEADER_TEMPLATE = "rep_header_stat_template.j2"
DEFAULT_SHIFT_STATISTICS_DATA_TEMPLATE = "rep_data_stat_template.j2"
DEFAULT_SEGMENT_HEADER_TEMPLATE = "rep_segment_header_template.j2"

###############################################################

class FttReportGenerator:
    def __init__(self, db_adapter, env, tile_server_info):
        self.db_adapter = db_adapter
        self.env = env
        self.map_generator = MapGenerator(
            db_adapter,
            tile_server_info['url'],
            tile_server_info['zoom_level'],
            builddir+"/"+imagedir, 
            builddir+"/"+maptiledir
        )

    @staticmethod
    def conv_time(seconds):
        m, s = divmod(seconds, 60)
        h, m = divmod(m, 60)
        return "%d:%02d:%02d" % (h, m, s)

    @staticmethod
    def get_bb_from_text(bb_str):
        bb = parse.parse("BOX({} {},{} {})", bb_str)
        bb = map(float, bb)
        return bb

    @staticmethod
    def text2latex(s):
        # Converts user text to LaTeX text
        if s is None:
            return None
        s = s.replace("\\", "\\backslash")
        s = s.replace("#", "\\#")
        s = s.replace("$", "\\$")
        s = s.replace("%", "\\%")
        s = s.replace("^", "\\^{}")
        s = s.replace("&", "\\&")
        s = s.replace("_", "\\_")
        s = s.replace("{", "\\{")
        s = s.replace("}", "\\}")
        s = s.replace("~", "\\~{}")
        s = s.replace("\xc3\x9f", "{\ss}") # replace ss
        s = s.replace("\xc3\xa4", "{\"a}") # replace ae
        s = s.replace("\xc3\x84", "{\"A}") # replace AE
        s = s.replace("\xc3\x9c", "{\"U}") # replace UE
        s = s.replace("\xc3\xbc", "{\"u}") # replace ue
        s = s.replace("\xc3\x96", "{\"O}") # replace OE
        s = s.replace("\xc3\xb6", "{\"o}") # replace oe
        return s

    def generate_latex_segments(self, latexf, shift_id, local):
        # Extract main info from master segments of selected shift.
        segment_values = self.db_adapter.get_master_segments(shift_id)
        # Write individual segment data to tex file.
        for row in segment_values:
            distance = self.db_adapter.get_segment_distance(row['segment_id'], local)
            # Get info from sub-segments for this master segment.
            data = self.db_adapter.get_sub_segments(row['segment_id'])
            # Format variables to write to tex template.
            weather = row['weather_short_description']
            if weather is None:
                weather = "--"
            duration = row['duration']
            if duration is None:
                duration = 0
            ito_reason = ""
            obstacle = ""
            lighting = ""
            slope = ""
            for row2 in data:
                if row2['short_description'] is not None:
                    if ito_reason:
                        ito_reason += ', '
                    ito_reason += '%s' % (row2['short_description'])
                if row2['obstacle'] is not None:
                    if obstacle:
                        obstacle += ', '
                    obstacle += '%s' % (row2['obstacle'])
                if row2['lighting'] is not None:
                    if lighting:
                        lighting += ', '
                    lighting += '%s' % (row2['lighting'])
                if row2['slope'] is not None:
                    if slope:
                        slope += ', '
                    slope += '%s' % (row2['slope'])
            if not ito_reason:
                ito_reason = '--'
            if not obstacle:
                obstacle = '--'
            if not lighting:
                lighting = '--'
            if not slope:
                slope = '--'
            # Get all notes for this master segment and its children (sub-segments).
            data = self.db_adapter.get_segment_notes(row['segment_id'])
            # Format notes to write to tex template.
            notes = ""
            for row2 in data:
                if notes:
                    notes += '; '
                notes += '%s: %s' % (self.text2latex(row2['name']), self.text2latex(row2['note']))
            if not notes:
                notes = '--'
            # Write variables to template and tex file.
            template = self.env.get_template(DEFAULT_SEGMENT_HEADER_TEMPLATE)
            latexf.write(
                template.render(segmentTypeShortDescription = row['segment_type_short_description'],
                                startDatetime = row['start_datetime'].strftime('%Y-%m-%d %H:%M:%S'),
                                origStartTimestamp = row['orig_start_timestamp'],
                                origStartDatetime = datetime.datetime.fromtimestamp(int(row['orig_start_timestamp'])).strftime('%Y-%m-%d %H:%M:%S') if row['orig_start_timestamp'] else "",
                                segmentId = row['segment_id'],
                                legNumber = row['leg_number'],
                                endDatetime = row['end_datetime'].strftime("%Y-%m-%d %H:%M:%S"),
                                duration = "{:8.2f}".format(duration), 
                                distance = "{:8.2f}".format(distance), 
                                obstacle = obstacle, 
                                lighting = lighting,
                                slope = slope, 
                                weather = weather,
                                itoReason = ito_reason,
                                notes = notes
                )
            )
            latexf.write('\n\n')
            # Get bounding box for the segment location points.
            data = self.db_adapter.get_segment_bb(row['segment_id'], local)
            bb_str = data[0]
            # Build map image.
            if bb_str is not None:
                seg_bb = self.get_bb_from_text(bb_str)
                if local:
                    self.map_generator.get_local_map(shift_id, row['segment_id'], seg_bb)
                else:
                    self.map_generator.get_map(shift_id, row['segment_id'], seg_bb)
            # Attach map image to tex file.
            image_path_file = "%s/%s/segment_%s.jpeg" % (builddir, imagedir, row['segment_id'])
            if os.path.isfile(image_path_file):
                latexf.write('\\vspace{0.2cm}\n')
                latexf.write('\\begin{center}\n\n')
                latexf.write('\\includegraphics[width=\maxwidth{16cm},height=15cm,keepaspectratio]{%s/segment_%s.jpeg}\n\n' % (imagedir, row['segment_id']))
                latexf.write('\\end{center}\n\n')
            else:
                latexf.write('(No map file available.)')
            latexf.write('\\newpage\n\n')
            # List all images for this master segment and its children
            data = self.db_adapter.get_segment_images(row['segment_id'])
            # Attach images and write description text to tex file.
            for count, row2 in enumerate(data):
                jpg_image_data = base64.standard_b64decode(row2['image'])
                if not (jpg_image_data.startswith('Error 404: Not Found')):
                    tmp_file = "%s/%s/%s_%s_cam.jpeg" % (builddir, imagedir, row2['image_segment_id'], count)
                    f = open(tmp_file, 'w')
                    f.write(jpg_image_data)
                    f.close()
                    latexf.write('\\vspace{0.2cm}\n')
                    latexf.write('\\begin{center}\n\n')
                    latexf.write(
                        '\\includegraphics[width=12cm,height=10cm,keepaspectratio]{%s/%s_%s_cam.jpeg}\n\n' % (
                            imagedir, row2['image_segment_id'], count))
                    if row2['description'] is not None and row2['description'] != "":
                        latexf.write('%s.\\\\\n' % (row2['description'].replace("_", "\_")))
                    else:
                        latexf.write('Image captured from the chase vehicle.')
                    latexf.write('\\end{center}\n\n')
                else:
                    print("Warning: Image data starts with 'Error 404: Not Found'")
            # End page for this segment.
            latexf.write('\\newpage\n\n')

    def generate_latex_shift_header(self, latexf, shift):
        print(" - Generating the Latex section for shift with id " + str(shift["id"]))
        # Chapter title.
        latexf.write('\\section{Shift %s: %s -- %s (%s)}\n\n' % (shift["id"], shift['shift_start_datetime'],
                                                                 shift['shift_end_datetime'],
                                                                 shift['performer_institution']))
        # Overview map.
        latexf.write('\\subsection{Overview map}\n\n')
        image_path_file = "{}/{}/shift_{}.jpeg".format(builddir, imagedir, shift["id"])
        latex_image_path_file = "{}/shift_{}.jpeg".format(imagedir, shift["id"])
        if os.path.isfile(image_path_file):
            latexf.write('The following map shows an overview of the whole shift.\n\n')
            latexf.write('\\vspace{0.5cm}\n')
            latexf.write('\\begin{center}\n\n')
            latexf.write('\\includegraphics[width=\maxwidth{16cm},height=15cm,keepaspectratio]{%s}\n\n' % latex_image_path_file)
            latexf.write('\\end{center}\n\n')
        else:
            latexf.write('(No overview map file available.)')
        # Shift summary.
        template = self.env.get_template(DEFAULT_SHIFT_HEADER_TEMPLATE)
        latexf.write(
            template.render(shiftStartDatetime=shift['shift_start_datetime'],
                            shiftEndDatetime=shift['shift_end_datetime'], id=shift['id'],
                            admin=shift['test_administrator_name'], director=shift['test_director_name'],
                            saftey=shift['safety_officer_name'], performer=shift['performer_institution'],
                            testIntent=shift['test_intent'], note=self.text2latex(shift['shift_note'])))
        latexf.write('\n\n')
        return

    def generate_latex_shift_statistic(self, latexf, shift_id, local):
        # Select the Shift statistics from the DB
        statistic = self.db_adapter.get_shift_statistics(shift_id, local)
        # Write ITO/AUTO statistics table.
        template = self.env.get_template(DEFAULT_SHIFT_STATISTICS_HEADER_TEMPLATE)
        latexf.write(
            template.render())
        for row in statistic:
            speed_m_s = 0
            if row['distance'] is not None and row['duration'] > 0:
                speed_m_s = row['distance'] / row['duration']
            speed_km_h = speed_m_s * 3.6
            speed_mi_h = speed_m_s * 2.236936292
            tmp_dur = FttReportGenerator.conv_time(row['duration'])
            template = self.env.get_template(DEFAULT_SHIFT_STATISTICS_DATA_TEMPLATE)
            latexf.write(
                template.render(segmentTypeShortDescription=row['segment_type_short_description'], Count=row['cnt'],
                                Duration=tmp_dur, Distance="{:8.2f}".format(row['distance']),
                                Speedm="{:3.2f}".format(speed_m_s), SpeedKm="{:3.2f}".format(speed_km_h),
                                SpeedMi="{:3.2f}".format(speed_mi_h)))
        latexf.write('\\end{longtable}\n')
        latexf.write('\n')

    def generate_latex_shift_timeline(self, latexf, shift, min_dur):
        # Get the shift timeline.
        shift_timeline = self.db_adapter.get_shift_timeline(shift['id'])
        # Generate the graphics.
        print("Shift timeline:")
        pattern = '%Y-%m-%d %H:%M:%S'
        shift_start_datetime = shift["shift_start_datetime"]
        shift_start_secs = int(time.mktime(time.strptime(shift_start_datetime, pattern)))
        print("shift_start_secs : %s" % (shift_start_secs,))
        shift_end_datetime = shift["shift_end_datetime"]
        shift_end_secs = int(time.mktime(time.strptime(shift_end_datetime, pattern)))
        print("shift_end_secs : %s" % (shift_end_secs,))

        # Make sure that the bar is at least n hours long
        t_min = float(min_dur) * 3600
        shift_duration_sec = shift_end_secs - shift_start_secs
        if shift_duration_sec < t_min:
            shift_duration_sec = t_min

        page_width_cm = 15.0
        cm_p_s = page_width_cm / shift_duration_sec
        latexf.write('\\subsection{Shift timeline}\n\n')
        latexf.write('\\begin{tikzpicture}[scale=1.0]\n')
        latexf.write('\\draw[->] (0,0) -- (%f,0);' % (shift_duration_sec * cm_p_s))
        max_lines = 16  # including the top lines AUTO / ITO
        start_flex_line = 5
        line = start_flex_line
        line_sep = -0.5
        t = 0
        last_weather_icon_filename = ""
        while t <= shift_duration_sec:
            abs_time = str(datetime.timedelta(seconds=t)).split('.', 2)[0] # H:MM:SS
            if shift_duration_sec < 3600:
                abs_time = abs_time[2:] + 's' # [2:] to remove the hours
            else:
                abs_time = abs_time[:-3] + 'm' # [:-3] to remove the seconds
            latexf.write('\\draw (%f,-0.1) -- (%f,0) node[anchor=south] {%s};\n' % (t * cm_p_s, t * cm_p_s, abs_time))
            t = t + shift_duration_sec / 10
        for row in shift_timeline:
            print("============== New segment ============")
            segment_type_key = row['segment_type_key']
            if (row['starttime'] is not None) and (shift['shift_start_datetime_raw'] is not None):
                start_secs = (row['starttime'] - shift["shift_start_datetime_raw"]).total_seconds()
            elif row['starttime'] is None:
                start_secs = 0
                print("Error Startitme is %s" % row['starttime'])
            else:
                start_secs = 0
                print("Error Datetime Raw is %s" % shift['shift_start_datetime_raw'])

            print("start_datetime. %s" % (row['starttime'],))
            print("start_datetime from shift. %s" % (start_secs,))
            if row['duration'] is not None:
                duration = row["duration"].total_seconds()
                print("duration: %s" % (duration,))
            else:
                print("Error Duration is unavible")

            segment_id = row['segment_id']
            segment_parent_id = row['segment_parent_id']
            segment_color = row['segment_color']
            ito_key = row['ito_key']
            ito_color = row['ito_color']
            ito_description = row['ito_description']
            if ito_description is None:
                ito_description = "--"
            if ito_description == "":
                ito_description = "--"
            weather_icon_filename = row['weather_icon_filename']
            is_master_seg = (segment_parent_id is None or segment_parent_id == '')
            if is_master_seg:
                jump_label = 'segment_%s' % segment_id
            else:
                jump_label = 'segment_%s' % segment_parent_id
            # determine y position and color
            if duration is None:
                duration = 0
            # print "Duration: ", duration
            if (duration * cm_p_s) > 0 or is_master_seg:
                # with duration or bar in upper line
                if is_master_seg:
                    y = 0
                    # Add weather icon
                    if weather_icon_filename is None:
                        weather_icon_filename = "unknown_weather"
                    if last_weather_icon_filename != weather_icon_filename:
                        latexf.write("\\pgftext[left, x=%fcm,y=%fcm] {\includegraphics[width=0.7cm]{%s/%s}}" % (
                            start_secs * cm_p_s, 1 * line_sep, logosdir, weather_icon_filename))
                        last_weather_icon_filename = weather_icon_filename
                    # master segment in upper rows
                    ito_description = ''
                    color = segment_color
                    if segment_type_key == 'auto':
                        y = 3 * line_sep
                    if segment_type_key == 'ito':
                        y = 4 * line_sep
                else:
                    # child segment in lower rows
                    color = ito_color
                    y = line * line_sep
                    line = line + 1
                    latexf.write('\\draw[dashed, color=lightgray] (%f,0) -- (%f,%f);\n' % (
                        start_secs * cm_p_s, start_secs * cm_p_s, y))
                latexf.write('\\definecolor{currentcolor}{RGB}{%s}\n' % color)
                latexf.write(
                    '\\draw[fill=currentcolor] (%f,%f-0.125) rectangle +(%f,0.25) node[anchor=west,yshift=-3pt]{' % (start_secs * cm_p_s, y, duration * cm_p_s)
                    + (('\\hyperref[%s]{%s}};\n' % (jump_label, ito_description)) if ito_description else '};\n'))
            else:
                # Segment without duration
                latexf.write('\\definecolor{currentcolor}{RGB}{%s}\n' % ito_color)
                if ito_key == 'goal' or ito_key == 'end_shift':
                    y = 2 * line_sep
                    latexf.write(
                        '\\draw[fill=currentcolor] (%f,%f) circle (.1cm) node[anchor=west,xshift=0.1cm,fill=white] {' % (start_secs * cm_p_s, y)
                        + (('\\hyperref[%s]{%s}};\n' % (jump_label, ito_description)) if ito_description else '};\n'))
                else:
                    y = line * line_sep
                    latexf.write(
                        '\\draw[fill=currentcolor] (%f,%f) circle (.1cm) node[anchor=west,xshift=1pt] {' % (start_secs * cm_p_s, y)
                        + (('\\hyperref[%s]{%s}};\n' % (jump_label, ito_description)) if ito_description else '};\n'))
                    latexf.write('\\draw[dashed, color=lightgray] (%f,0) -- (%f,%f);\n' %
                                 (start_secs * cm_p_s, start_secs * cm_p_s, y))
                    line = line + 1
            if line > max_lines:
                line = start_flex_line
        latexf.write('\\draw[->] (0,%f) -- (%f,%f);\n' % (
            (max_lines + 1) * line_sep, shift_duration_sec * cm_p_s, (max_lines + 1) * line_sep))
        t = 0
        while t <= shift_duration_sec:
            abs_time = t + int(shift_start_secs)
            abs_time_str = datetime.datetime.utcfromtimestamp(abs_time).strftime('%H:%M')
            latexf.write('\\draw (%f,0) -- (%f,%f) node[anchor=north] {%s};\n' % (
                t * cm_p_s, t * cm_p_s, (max_lines + 1) * line_sep - 0.1, abs_time_str))
            t = t + 900  # 900 sec = 15 min
        latexf.write('\\end{tikzpicture}\n')
        latexf.write('\\newpage\n\n')

    # Write latex figure.
    def write_latex_figure(self, latex_file, figure, short_caption, long_caption):
        image_path_file = "{}/{}/{}.png".format(builddir, imagedir, figure)
        latex_image_path_file = "{}/{}.png".format(imagedir, figure)

        if os.path.isfile(image_path_file):    
            latex_file.write('\\begin{figure}[H]\n')
            latex_file.write('\\begin{center}\n')
            # latex_file.write('\\vspace{-0.5cm}\n')
            latex_file.write('\\includegraphics[height=9cm, trim = {0 0 0 1cm}, clip]{%s}\n' % latex_image_path_file)
            latex_file.write('\\setlength{\\belowcaptionskip}{-10pt}')
            latex_file.write('\\caption[%s]{%s}\n' % (short_caption, long_caption))
            latex_file.write('\\label{fig:%s}\n' % figure)
            latex_file.write('\\end{center}\n')
            latex_file.write('\\end{figure}\n')

    # Generate the global plots for the test event and write them to the latex file.
    def generate_latex_global_statistics(self, latexf, shifts, report_info):
        # Create the plots.
        global_plots = PlotsGenerator(self.db_adapter, report_info['test_event_id'],shifts, report_info['local'], builddir, builddir+'/'+imagedir, logosdir, 'png')
        global_plots.generate_all_plots()
        # Define figure names.
        total_dist_figure = 'TotalAutoDist'
        mean_dist_figure = 'MeanAutoDist'
        stop_counts_figure = 'OverrideCounts'
        manual_figure = 'ManualTimes'
        # Write to the Latex file.
        print(" - Generating the Latex section for global statistics")
        # Chapter title.
        latexf.write('\\section{Global statistics}\n\n')
        latexf.write('\\subsection{Autonomously travelled distance}\n\n')
        latexf.write('Figure \\ref{fig:%s} and Figure \\ref{fig:%s} summarize the performance of autonomous driving over the different test shifts.' % (total_dist_figure, mean_dist_figure))
        # Plots.
        short_caption = 'Total autonomous distance.'
        long_caption = 'Total distance driven autonomously. \
            Each bar section within a shift represents an autonomous segment. \
            The total autonomous drive distance is shown above each shift. \
            The icon(s) represents the weather condition during the shift.'
        self.write_latex_figure(latexf, total_dist_figure, short_caption, long_caption)

        short_caption = 'Mean autonomous distance.'
        long_caption = 'Mean distance driven autonomously. \
            The circle at the bottom of each bar shows the mean autonomous drive distance. \
            The upper bar line shows the maximum autonomous drive distance. \
            The icon(s) represents the weather condition during the shift.'
        self.write_latex_figure(latexf, mean_dist_figure, short_caption, long_caption)

        latexf.write('\\subsection{Stop events and manual operation}\n\n')
        latexf.write('Figure \\ref{fig:%s} and Figure \\ref{fig:%s} summarize the occurrence of stops and the spent time in manual operation.' % (stop_counts_figure, manual_figure))

        short_caption = 'Override counts.'
        long_caption = 'Counts for the different ITO reasons (see legend). \
            The number above each shift indicates the total manual drive distance in meters. \
            The icon(s) represents the weather condition during the shift.'
        self.write_latex_figure(latexf, stop_counts_figure, short_caption, long_caption)

        short_caption = 'Manual times.'
        long_caption = 'Time spent in manual operation. \
            The total manual time is shown above each shift.'
        self.write_latex_figure(latexf, manual_figure, short_caption, long_caption)
        
        latexf.write ('\\clearpage\n\n') # clearpage flushes all pending floats

    def generate_latex_header(self, latexf, report_info):
        print(">>> Generating the Latex files for " + report_info["test_event_name"])
        template = self.env.get_template(report_info["latex_template"])
        latexf.write(template.render(
            latexTestCampaignName = report_info["test_event_name"], 
            version = report_info["report_version"],
            recipientName = report_info["recipient_name"],
            recipientAddress = report_info["recipient_address"],
            creatorName = report_info["creator_name"],
            creatorAddress = report_info["creator_address"],
            topLogoPath = report_info["top_logo_path"],
            bottomLogoPath = report_info["bottom_logo_path"]
        ))
        return latexf

    @staticmethod
    def generate_latex_end(latexf):
        latexf.write('''\\end{document}''')
        latexf.close()

    def generate_latex_report(self, report_info):
        # Create the Filepath + Name of the .tex-File to be written
        filename = '%s/%s.tex' % (builddir, report_info["test_event_filename"])
        # Open the File in write Mode
        latex_f = open(filename, 'w')
        print("Writing report %s" % (filename,))
        # Generate the Latex file header
        self.generate_latex_header(latex_f, report_info)
        # Get the Shift entries out of the DB
        shifts = self.db_adapter.get_shifts_by_test_event_id(report_info["test_event_id"], report_info["local"])
        # Generate the global statistics section
        self.generate_latex_global_statistics(latex_f, shifts, report_info) # list(map(lambda x: x['id'], shifts))

        # Iterate over each Shift
        for shift in shifts:
            # With shift Id
            shift_id = shift["id"]
            # Start time of the selected Shift
            shift_start_datetime = shift["shift_start_datetime"]
            # End time of the selectedShift
            shift_end_datetime = shift["shift_end_datetime"]
            # Boundingbox of the Shift text
            bb_str = shift["shift_bb"]
            print("============== New Shift %s ============" % shift_id)
            print("Adding Shift %s (%s, %s)" % (shift_id, shift_start_datetime, shift_end_datetime))

            if not (bb_str is None):
                bb = self.get_bb_from_text(bb_str)
                # Get the Map and will save it to ~/build/images/
                if report_info["local"]:
                    # Get map from local position data.
                    self.map_generator.get_local_map(shift_id, None, bb)
                else:
                    # Get map from GPS data.
                    self.map_generator.get_map(shift_id, None, bb)
            # Generate the Latex Shift header
            self.generate_latex_shift_header(latex_f, shift)
            # Generate the Shift statistics for a Latex file
            self.generate_latex_shift_statistic(latex_f, shift_id, report_info["local"])
            # Create the Timeline for the Shift
            self.generate_latex_shift_timeline(latex_f, shift, report_info['min_duration'])
            # Create the Segment sections
            self.generate_latex_segments(latex_f, shift_id, report_info["local"])

        self.generate_latex_end(latex_f)
        return

class ReportGenerator:

    def init_dirs(self):
        img_dir_str = "%s/%s" % (builddir, imagedir)
        map_dir_str = "%s/%s" % (builddir, maptiledir)
        if not os.path.exists(builddir):
            os.makedirs(builddir)
        if not os.path.exists(img_dir_str):
            os.makedirs(img_dir_str)
        if not os.path.exists(map_dir_str):
            os.makedirs(map_dir_str)
        return


    def read_xml(self, filename):
        # Initialize return objects
        report_info_list = []
        db_info = {}
        tile_server_info = {}
        # Open and parse XML.
        with open(filename, 'rb') as xml_file:
            root = etree.parse(xml_file).getroot()
        # Retrieve data from XML.
        for child in root:
            # Get tile server name.
            if child.tag == "resource":
                tile_server_info['url'] = child.get("tile_server")
                try:
                    tile_server_info['zoom_level'] = int(child.get("zoom_level"))
                except ValueError:
                    tile_server_info['zoom_level'] = 19
            # Get technical db info.
            if child.tag == "postgis":
                db_info['dbname'] = child.get("dbname")
                db_info['host'] = child.get("host")
                db_info['user'] = child.get("user")
                db_info['password'] = child.get("password")
            # Get report info.
            if child.tag == "report":
                report_info = {}
                report_info['test_event_name'] = child.get("name")
                report_info['report_version'] = child.get("version")
                report_info['test_event_filename'] = "report" #report_info['test_event_name'].lower().replace(" ", "_")
                for k in child:
                    if k.tag == "test_event":
                        report_info['test_event_id'] = k.get("id")
                        report_info['min_duration'] = k.get("min_dur")
                        report_info['local'] = k.get("local").lower() == "true"
                    if k.tag == "recipient":
                        report_info['recipient_name'] = k.get("name")
                        report_info['recipient_address'] = k.get("address")
                    if k.tag == "creator":
                        report_info['creator_name'] = k.get("name")
                        report_info['creator_address'] = k.get("address")
                    if k.tag == "logos":
                        report_info['top_logo_path'] = k.get("top_logo_path")
                        report_info['bottom_logo_path'] = k.get("bottom_logo_path")
                report_info_list.append(report_info)

        return report_info_list, db_info, tile_server_info

    def generate(self, argv):
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        self.init_dirs()
        # Parse the XML-File and save the Values to corresponding Vars
        (report_info_list, db_info, tile_server_info) = self.read_xml(argv[0])

        # Read connection details from the XML-File-Var and connect to the DB
        ftt_adapter = FttAdapter(db_info['host'], db_info['dbname'], db_info['user'], db_info['password'])
        ftt_adapter.connect()

        # Init the ReportGenerator
        env = Environment(
            variable_start_string = '\VAR{',
	        variable_end_string = '}',
            loader=FileSystemLoader(dname+'/templates/')
        )
        ftt_report_generator = FttReportGenerator(ftt_adapter, env, tile_server_info)

        for report_info in report_info_list:
            report_info["latex_template"] = DEFAULT_HEADER_LATEX_TEMPLATE
            ftt_report_generator.generate_latex_report(report_info)
            call_str = '%s.tex' % (report_info['test_event_filename'])
            main_path = os.getcwd()
            os.chdir(builddir)
            call(['pdflatex', call_str])
            call(['pdflatex', call_str])
            file_path = os.path.normpath(report_info['test_event_filename'] + '.pdf')
            abs_file_path = os.path.abspath(file_path)
            call(['cp', abs_file_path, builddir])
            os.chdir(main_path)

if __name__ == "__main__":
    report_generator = ReportGenerator()
    report_generator.generate(sys.argv[1:])