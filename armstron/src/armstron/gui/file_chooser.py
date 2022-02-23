from gi.repository import Gtk
import os
import copy

class FileChooserWindow():
    def __init__(self, start_file, filters):
        self.filename=copy.deepcopy(start_file)
        self.filters = filters
        print('starting here:' ,self.filename)

        self.win = Gtk.Window(title="FileChooser")

        box = Gtk.Box(spacing=6)
        self.win.add(box)
        button1 = Gtk.Button("Choose File")
        button1.connect("clicked", lambda widget : self.win.close())
        box.add(button1)

        self.win.connect("delete-event", Gtk.main_quit)
        self.win.show_all()
        Gtk.main()


    def get_filename(self):
        return copy.deepcopy(self.filename)
    

    def ask_which(self):
        box = Gtk.Box(spacing=6)
        self.win.add(box)

        button1 = Gtk.Button("Choose File")
        button1.connect("clicked", self.open_file)
        box.add(button1)

        button2 = Gtk.Button("Choose Folder")
        button2.connect("clicked", self.open_folder)
        box.add(button2)


    def open_file(self, widget=None):
        dialog = Gtk.FileChooserDialog("Please choose a file", self.win,
            Gtk.FileChooserAction.OPEN,
            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            Gtk.STOCK_OPEN, Gtk.ResponseType.OK))

        dialog.connect("delete-event", Gtk.main_quit)
       
        if self.filename['dirname'] is not None:
            dialog.set_current_folder(self.filename['dirname'])

        self.add_filters(dialog)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            filename= dialog.get_filename()

            self.filename['dirname']=os.path.dirname(filename)
            self.filename['basename']=os.path.basename(filename)
            print("Open clicked")
            print("File selected: " + filename)
        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")

        
        dialog.destroy()
        self.win.destroy()
        


    def save_file(self, widget=None):
        dialog = Gtk.FileChooserDialog("Please choose a file", self.win,
            Gtk.FileChooserAction.SAVE,
            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            Gtk.STOCK_SAVE, Gtk.ResponseType.OK))

        dialog.connect("delete-event", Gtk.main_quit)

        if self.filename['basename'] is not None:
            dialog.set_current_name(self.filename['basename'])
        
        if self.filename['dirname'] is not None:
            dialog.set_current_folder(self.filename['dirname'])

        self.add_filters(dialog)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            filename= dialog.get_filename()

            self.filename['dirname']=os.path.dirname(filename)
            self.filename['basename']=os.path.basename(filename)
            print("Save clicked")
            print("File selected: " + filename)
        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")

   
        dialog.destroy()
        self.win.destroy()
        


    def add_filters(self, dialog):
        for filter in self.filters:
            filter = list(filter)
            if "*" not in filter[1]:
                filter[1] = "*"+filter[1]
            elif filter[1] == ".*":
                filter[1] = '*'
            
            print(filter)
            filter_text = Gtk.FileFilter()
            filter_text.set_name(filter[0])
            filter_text.add_pattern(filter[1])
            dialog.add_filter(filter_text)


    def open_folder(self, widget=None):
        dialog = Gtk.FileChooserDialog("Please choose a folder", self.win,
            Gtk.FileChooserAction.SELECT_FOLDER,
            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            "Select", Gtk.ResponseType.OK))
        dialog.set_default_size(700, 400)

        dialog.connect("delete-event", Gtk.main_quit)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            print("Select clicked")
            print("Folder selected: " + dialog.get_filename())
        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")

        dialog.destroy()
        self.win.destroy()

    
    def shutdown(self):
        self.win.destroy()