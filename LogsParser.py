import json
import csv
import re

input_file_path = "raw_logs/07-20_12-57-56.json"  # Path to the input JSON file
def filter_json_file(allowedTypes, writingColumns, overwrite=False):
    global input_file_path
    lines = []
    with open(input_file_path, 'r') as json_file:
        lines = json_file.readlines()

    filtered_lines = [line.strip() for line in lines if line.split(', ')[0].removeprefix('type: ') in allowedTypes]
    if overwrite is True:
        with open(input_file_path, 'w') as json_file:
            json_file.write('\n'.join(filtered_lines))


    csv_file_path = input_file_path.replace('.json', '.csv')
    with open(csv_file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(writingColumns)
        for line in filtered_lines:
            line_data = line.split(', ')
            if float(re.sub(r'[^0-9(),.-]', '', line_data[1])) < -180:
                continue
            # values = [re.sub(r'[^0-9(),.-]', '', line_data[1]), re.sub(r'[^0-9(),.-]', '', line_data[2]), re.sub(r'[^0-9(),.-]', '', line_data[3]), re.sub(r'[^0-9(),.-]', '', str(line_data[4:]))]
            values = [re.sub(r'[^0-9(),.-]', '', line_data[1]), re.sub(r'[^0-9(),.-]', '', line_data[2])]
            # values = [line_data[0].removeprefix('type: '), re.sub(r'[^0-9(),.-]', '', str(line_data[4] + ", " + line_data[5]))]
            writer.writerow(values)

# Usage example
value = ['Raw']
included = ['longitude', 'latitude']
filter_json_file(value, included, overwrite=False)