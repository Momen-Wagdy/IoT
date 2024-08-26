import http.server
import socketserver
import cgi
import os

PORT = 19999
UPLOAD_DIR = r"/home/bruh/espimages"
# os.mkdir(UPLOAD_DIR)
INDEX_FILE = "index.html"  # Make sure this file exists in the current directory

class SimpleHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":  # Serve the index.html file for the root path
            self.path = INDEX_FILE
        return super().do_GET()

    def do_POST(self):
        # Parse the form data posted
        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={'REQUEST_METHOD': 'POST'}
        )

        # Extract the uploaded file
        file_field = form['imageFile']  # 'file' should match the key in the HTML form
        if file_field.filename:
            # Ensure the upload directory exists
            if not os.path.exists(UPLOAD_DIR):
                os.makedirs(UPLOAD_DIR)

            # Create the full file path
            file_path = os.path.join(UPLOAD_DIR, file_field.filename)

            # Save the file to the server
            with open(file_path, 'wb') as output_file:
                output_file.write(file_field.file.read())

            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"File uploaded successfully")
        else:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"No file uploaded")

if __name__ == "__main__":
    with socketserver.TCPServer(("", PORT), SimpleHTTPRequestHandler) as httpd:
        print(f"Serving on port {PORT}")
        httpd.serve_forever()
