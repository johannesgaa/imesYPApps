package de.unihannover.imes.imesypmesse;


import android.app.Activity;
import android.os.Bundle;
import android.widget.Gallery;
import android.widget.ImageView;

public class MainActivity extends Activity
{
	//variable for selection intent
  	private final int PICKER = 1;
	//variable to store the currently selected image
	private int currentPic = 0;
	//gallery object
	@SuppressWarnings("deprecation")
	private Gallery picGallery;
	//image view for larger display
	private ImageView picView;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
      //get the large image view
        picView = (ImageView) findViewById(R.id.picture);
        //get the gallery view
        picGallery = (Gallery) findViewById(R.id.gallery1);
        
    }
}
